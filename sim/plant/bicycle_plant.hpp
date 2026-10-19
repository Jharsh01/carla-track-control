// Bicycle plant: the bicycle model plus effects the controller model leaves out.
//
// On top of `dynamics` the plant adds
// - actuator saturation (steering angle and rate, tire and engine force limits),
// - a first-order steering actuator lag,
// - first-order tire force build-up (relaxation length),
// and it can be given parameters that differ from the controller's.
//
// The plant runs either offline, stepped by its caller (`step`), or in real
// time on its own thread (`start`), at a fixed rate (1 kHz by default). The
// thread is woken by a periodic kernel timer (timerfd, CLOCK_MONOTONIC): the
// high-resolution timer interrupt fires at absolute deadlines start + k *
// period, so the rate does not drift however long a step takes. A tick that
// comes late is not lost: the timer counts the periods since the last wake-up
// and the plant steps through all of them, so plant time stays wall-clock time
// since `start`; such ticks count as overruns (`timing`). The thread asks for
// SCHED_FIFO, which needs root, CAP_SYS_NICE or an rtprio limit (ulimit -r),
// and otherwise runs at normal priority; only a PREEMPT_RT kernel bounds the
// wake-up latency.
//
// The running plant shares its signals through the global `bicycle_state`:
//
//   double t;
//   const State x = bicycle_state.state(&t);  // a consistent [X, Y, psi, Ux, Uy, r, delta] and its time
//   const double ux = bicycle_state[IUX];     // or one state variable
//   bicycle_state.command(u);                 // held by the plant from its next tick on
//   bicycle_state.mu = 0.9;                   // friction under the car, from its next tick on
//                                             // (or the plant looks it up itself: surface_friction)
//
// No call waits on the plant thread. While the thread runs it owns the plant:
// go through `bicycle_state`, not `step`, `observe`, `reset` or `saturate`.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

#include "sim/plant/interface.hpp"

namespace sim {

// N doubles with one writer that never waits and readers that see a whole write or none of it.
template <int N>
class SeqLock {
public:
    void write(const std::array<double, N>& v) {
        const uint64_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);  // odd: a write is under way
        std::atomic_thread_fence(std::memory_order_release);
        for (int i = 0; i < N; ++i) data_[i].store(v[i], std::memory_order_relaxed);
        seq_.store(s + 2, std::memory_order_release);
    }

    std::array<double, N> read() const {
        std::array<double, N> v;
        for (;;) {
            const uint64_t s = seq_.load(std::memory_order_acquire);
            if (s & 1) continue;
            for (int i = 0; i < N; ++i) v[i] = data_[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s) return v;
        }
    }

    double get(int i) const { return data_[i].load(std::memory_order_relaxed); }

private:
    std::atomic<uint64_t> seq_{0};
    std::array<std::atomic<double>, N> data_{};
};

// The signals of the running bicycle plant, shared with the controllers.
struct BicycleBus {
    SeqLock<1 + NX> sample;  // [t, X, Y, psi, Ux, Uy, r, delta] after the latest tick, t in plant time [s]
    SeqLock<NU> input;  // commanded [steering rate, Fx]
    SeqLock<NU> applied;  // the command after the plant's limits, as applied at the latest tick
    std::atomic<double> mu{1.0};  // tire-road friction under the car

    State state(double* t = nullptr) const {
        const auto v = sample.read();
        if (t) *t = v[0];
        return Eigen::Map<const State>(v.data() + 1);
    }
    double operator[](int i) const { return sample.get(1 + i); }  // one state variable, e.g. bicycle_state[IUX]
    double time() const { return sample.get(0); }
    void command(const Input& u) { input.write({u[IRATE], u[IFX]}); }
    Input commanded() const { return as_input(input.read()); }
    Input applied_input() const { return as_input(applied.read()); }

    void publish(double t, const State& x) {
        std::array<double, 1 + NX> v;
        v[0] = t;
        for (int i = 0; i < NX; ++i) v[1 + i] = x[i];
        sample.write(v);
    }

private:
    static Input as_input(const std::array<double, NU>& v) { return {v[IRATE], v[IFX]}; }
};

// Written by the running BicyclePlant; one plant may run at a time.
extern BicycleBus bicycle_state;

class BicyclePlant : public PlantModel {
public:
    // `tau_steer` [s] and `relaxation_length` [m] of 0 switch the respective lag off.
    explicit BicyclePlant(const VehicleParams& params, double dt = 1e-3, double tau_steer = 0.05,
                          double relaxation_length = 0.3);
    ~BicyclePlant() override;  // stops the thread

    void reset(const State& x0) override;
    State observe() const override { return z_.head<NX>(); }
    Input saturate(const Input& u) const override;
    State step(const Input& u, double duration) override;  // throws std::logic_error while running
    void set_friction(double mu) override { p.mu = mu; }

    // ---- real time

    struct Realtime {
        double rate = 1000.0;  // [Hz]; a whole number of time steps dt per period
        int priority = 80;  // SCHED_FIFO priority 1..99; 0: normal scheduling
        int cpu = -1;  // core to pin the thread to; -1: any
    };

    struct TimingStats {
        uint64_t ticks;  // timer periods since start
        uint64_t overruns;  // of them, periods the thread woke up too late to step on time
        double latency_mean, latency_max;  // from a deadline to the thread waking up [s]
        double step_max;  // longest tick [s]
        bool realtime;  // runs under SCHED_FIFO
    };

    // Publish the current state at t = 0 to `bicycle_state` and start ticking with
    // its `mu` and commanded input; set those before.
    void start(const Realtime& options);
    void start() { start(Realtime()); }
    void stop();  // at the next tick
    bool running() const;
    TimingStats timing() const;

    // Friction of the surface under the car, for the running plant to apply every tick (and publish as
    // bicycle_state.mu) instead of taking bicycle_state.mu; nan (off the map) keeps the last value. Called
    // on the plant's thread: it must be quick and safe to call from there.
    std::function<double(const State&)> surface_friction;

    VehicleParams p;
    double dt, tau_steer, relaxation_length;

private:
    // internal state: model state, steering reference, lagged slip angles
    using Internal = Eigen::Matrix<double, NX + 3, 1>;
    Internal deriv(const Internal& z, const Input& u) const;
    void advance(const Input& u, int steps);  // `steps` time steps holding u
    void tick_loop();

    Internal z_;
    struct Runner;  // the real-time thread and its timing
    std::unique_ptr<Runner> runner_;
};

}  // namespace sim
