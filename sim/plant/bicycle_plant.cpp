#include "sim/plant/bicycle_plant.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <string>
#include <thread>

#include "track_control/model/bicycle.hpp"
#include "sim/plant/filters.hpp"
#include "track_control/model/integrate.hpp"

namespace sim {

// ---- the shared signals and the real-time thread's bookkeeping

BicycleBus bicycle_state;

namespace {

std::atomic<bool> bus_owned{false};  // a running plant writes bicycle_state; only one may

int64_t now_ns() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return int64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}

timespec to_timespec(int64_t ns) { return {static_cast<time_t>(ns / 1000000000), static_cast<long>(ns % 1000000000)}; }

void store_max(std::atomic<double>& m, double v) {  // one writer
    if (v > m.load(std::memory_order_relaxed)) m.store(v, std::memory_order_relaxed);
}

}  // namespace

struct BicyclePlant::Runner {
    Realtime options;
    int substeps;  // time steps dt per period
    int64_t period_ns;
    int timer = -1;
    std::thread thread;
    std::atomic<bool> running{false}, ready{false}, realtime{false};
    std::atomic<uint64_t> ticks{0}, overruns{0};
    std::atomic<double> latency_sum{0.0}, latency_max{0.0}, step_max{0.0};
};

// ---- dynamics

BicyclePlant::BicyclePlant(const VehicleParams& params, double dt, double tau_steer, double relaxation_length)
    : p(params), dt(dt), tau_steer(tau_steer), relaxation_length(relaxation_length) {
    reset(State::Zero());
}

BicyclePlant::~BicyclePlant() { stop(); }

void BicyclePlant::reset(const State& x0) {
    const auto alpha = slip_angles(x0, p);
    z_ << x0, x0[IDELTA], alpha[0], alpha[1];
}

Input BicyclePlant::saturate(const Input& u) const {
    const auto [fx_min, fx_max] = fx_limits(z_[IUX], p);
    return {std::clamp(u[IRATE], -p.delta_rate_max, p.delta_rate_max), std::clamp(u[IFX], fx_min, fx_max)};
}

BicyclePlant::Internal BicyclePlant::deriv(const Internal& z, const Input& u) const {
    const State x = z.head<NX>();
    const double delta_ref = z[NX];
    const std::array<double, 2> alpha_lag = {z[NX + 1], z[NX + 2]};

    double ddelta_ref = u[IRATE];
    if (std::fabs(delta_ref) >= p.delta_max && ddelta_ref * delta_ref > 0.0) ddelta_ref = 0.0;
    const double ddelta = tau_steer > 0.0 ? first_order_lag(x[IDELTA], delta_ref, tau_steer) : ddelta_ref;

    const std::array<double, 2> alpha_ss = slip_angles(x, p);
    std::array<double, 2> alpha = alpha_ss, dalpha = {0.0, 0.0};
    if (relaxation_length > 0.0) {
        alpha = alpha_lag;
        const double speed = std::max(x[IUX], p.Ux_min);
        for (int i = 0; i < 2; ++i) dalpha[i] = relaxation(alpha_lag[i], alpha_ss[i], speed, relaxation_length);
    }

    const StateOf<double> dx = dynamics<double>(to_array(x), {ddelta, u[IFX]}, p, &alpha);
    Internal dz;
    dz << to_state(dx), ddelta_ref, dalpha[0], dalpha[1];
    return dz;
}

void BicyclePlant::advance(const Input& u, int steps) {
    for (int i = 0; i < steps; ++i)
        z_ = rk4_step([this](const Internal& z, const Input& v) { return deriv(z, v); }, z_, saturate(u), dt);
}

// ---- offline: stepped by the caller

State BicyclePlant::step(const Input& u, double duration) {
    if (running()) throw std::logic_error("BicyclePlant::step: the plant is running on its thread");
    advance(u, static_cast<int>(std::lround(duration / dt)));
    return observe();
}

// ---- real time: stepped on its own thread by a periodic timer

void BicyclePlant::start(const Realtime& options) {
    if (running()) return;
    const double steps = 1.0 / (options.rate * dt);
    if (!(options.rate > 0.0) || steps < 0.5 || std::fabs(steps - std::round(steps)) > 1e-6)
        throw std::invalid_argument("BicyclePlant::start: the period 1/rate must be a whole number of time steps dt");
    if (bus_owned.exchange(true)) throw std::runtime_error("BicyclePlant::start: another plant is running");

    auto runner = std::make_unique<Runner>();
    runner->options = options;
    runner->substeps = static_cast<int>(std::lround(steps));
    runner->period_ns = std::llround(1e9 / options.rate);
    runner->timer = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (runner->timer < 0) {
        bus_owned = false;
        throw std::runtime_error(std::string("BicyclePlant::start: timerfd_create: ") + std::strerror(errno));
    }
    runner_ = std::move(runner);
    bicycle_state.publish(0.0, observe());
    bicycle_state.applied.write({0.0, 0.0});
    runner_->running = true;
    runner_->thread = std::thread([this] { tick_loop(); });
    while (!runner_->ready) std::this_thread::yield();
}

void BicyclePlant::stop() {
    if (!runner_ || !runner_->thread.joinable()) return;
    runner_->running = false;
    runner_->thread.join();
    close(runner_->timer);
    runner_->timer = -1;
    bus_owned = false;
}

bool BicyclePlant::running() const { return runner_ && runner_->running; }

BicyclePlant::TimingStats BicyclePlant::timing() const {
    if (!runner_) return {0, 0, 0.0, 0.0, 0.0, false};
    const Runner& r = *runner_;
    const uint64_t ticks = r.ticks;
    return {ticks, r.overruns, ticks ? r.latency_sum / ticks : 0.0, r.latency_max, r.step_max, r.realtime};
}

void BicyclePlant::tick_loop() {
    Runner& r = *runner_;
    pthread_setname_np(pthread_self(), "plant");  // as top -H shows it
    if (r.options.cpu >= 0) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(r.options.cpu, &cpus);
        pthread_setaffinity_np(pthread_self(), sizeof cpus, &cpus);
    }
    if (r.options.priority > 0) {
        sched_param param{};
        param.sched_priority =
            std::clamp(r.options.priority, sched_get_priority_min(SCHED_FIFO), sched_get_priority_max(SCHED_FIFO));
        r.realtime = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
    }

    // first deadline one period from now, then one every period, on absolute time
    const int64_t start = now_ns();
    itimerspec spec{};
    spec.it_interval = to_timespec(r.period_ns);
    spec.it_value = to_timespec(start + r.period_ns);
    timerfd_settime(r.timer, TFD_TIMER_ABSTIME, &spec, nullptr);
    r.ready = true;

    const double period = 1e-9 * r.period_ns;
    uint64_t tick = 0;
    while (r.running) {
        uint64_t expirations = 0;  // periods since the last wake-up, 1 when on time
        if (read(r.timer, &expirations, sizeof expirations) != sizeof expirations) continue;  // EINTR
        const int64_t woke = now_ns();
        tick += expirations;

        if (surface_friction) {
            const double mu = surface_friction(observe());
            if (!std::isnan(mu)) bicycle_state.mu.store(mu, std::memory_order_relaxed);
        }
        set_friction(bicycle_state.mu.load(std::memory_order_relaxed));
        const Input u = bicycle_state.commanded();
        const Input applied = saturate(u);
        bicycle_state.applied.write({applied[IRATE], applied[IFX]});
        advance(u, static_cast<int>(expirations) * r.substeps);  // late: catch up to wall-clock time
        bicycle_state.publish(tick * period, observe());

        const double latency = 1e-9 * double(woke - (start + int64_t(tick) * r.period_ns));
        r.ticks.store(tick, std::memory_order_relaxed);
        r.overruns.fetch_add(expirations - 1, std::memory_order_relaxed);
        r.latency_sum.store(r.latency_sum.load(std::memory_order_relaxed) + latency, std::memory_order_relaxed);
        store_max(r.latency_max, latency);
        store_max(r.step_max, 1e-9 * double(now_ns() - woke));
    }
}

}  // namespace sim
