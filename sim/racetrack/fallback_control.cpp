#include "sim/racetrack/fallback_control.hpp"

#include <pthread.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "sim/plant/bicycle_plant.hpp"
#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"

namespace sim {

namespace {

tc::NMPC& nmpc_of(Driver& driver) {
    auto* nmpc = dynamic_cast<tc::NMPC*>(&driver.controller());
    if (!nmpc) throw std::invalid_argument("FallbackControl: the first driver must be the NMPC");
    return *nmpc;
}

// The state `lead` seconds on from x, at plant time t, with the (already limited) input u_at(time).
template <class InputAt>
State predict(const State& x, double t, InputAt u_at, const VehicleParams& p, double lead) {
    const int steps = std::max(1, static_cast<int>(std::lround(lead / 0.005)));
    const double dt = lead / steps;
    State z = x;
    for (int i = 0; i < steps; ++i) {
        const Input u = u_at(t + i * dt);
        z = rk4_step([&](const State& s, const Input& v) { return dynamics(s, v, p); }, z, u, dt);
    }
    return z;
}

}  // namespace

const char* source_name(FallbackControl::Source source) {
    switch (source) {
        case FallbackControl::Source::nmpc: return "nmpc";
        case FallbackControl::Source::pure_pursuit: return "pure_pursuit";
        case FallbackControl::Source::blend: return "blend";
        default: return "none";
    }
}

FallbackControl::FallbackControl(Driver& nmpc, Driver& pure_pursuit, Options options)
    : nmpc_driver_(nmpc), pure_pursuit_driver_(pure_pursuit), nmpc_(nmpc_of(nmpc)), options_(options) {
    if (!(options.rate > 0.0) || !(options.period > 0.0) || !(options.blend >= 0.0))
        throw std::invalid_argument("FallbackControl: rate and period must be positive, blend not negative");
}

void FallbackControl::start() {
    if (running_) return;
    running_ = true;
    threads_.emplace_back([this] { nmpc_loop(); });
    threads_.emplace_back([this] { pure_pursuit_loop(); });
    threads_.emplace_back([this] { command_loop(); });
}

void FallbackControl::stop() {
    running_ = false;
    for (std::thread& thread : threads_) thread.join();  // the NMPC thread finishes its solve first
    threads_.clear();
}

FallbackControl::Status FallbackControl::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

std::vector<std::pair<double, double>> FallbackControl::plan() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::pair<double, double>> out;
    if (current_.valid)
        for (int k = 0; k < current_.path.rows(); ++k) out.emplace_back(current_.path(k, IX), current_.path(k, IY));
    return out;
}

void FallbackControl::nmpc_loop() {
    pthread_setname_np(pthread_self(), "nmpc");  // as top -H shows it
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options_.period));
    Clock::time_point next = Clock::now();
    while (running_) {
        double t;
        const State x = bicycle_state.state(&t);
        const Input applied = bicycle_state.applied_input();
        const double t0 = t + options_.period;  // the plan starts where this period ends
        // Until then the car gets the plan in use (or the one about to take over), if the NMPC alone is
        // commanding, and otherwise about the input it has now.
        Plan now, coming;
        bool nmpc_alone;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            now = current_;
            if (has_next_ && next_.valid) coming = next_;
            nmpc_alone = status_.weight == 0.0;
        }
        auto u_at = [&](double time) -> Input {
            const Plan& plan = coming.valid && time >= coming.t0 ? coming : now;
            const int k = plan.valid ? tc::step_at(plan.times, time - plan.t0) : -1;
            if (nmpc_alone && k >= 0 && k < plan.inputs.rows()) return plan.inputs.row(k).transpose();
            return applied;
        };
        const State x0 = predict(x, t, u_at, nmpc_.p, options_.period);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            solving_ = true;
            solving_t0_ = t0;
        }
        const Clock::time_point start = Clock::now();
        nmpc_driver_(t0, x0);
        const double took = std::chrono::duration<double>(Clock::now() - start).count();
        const bool converged = nmpc_.last_success;
        // a converged plan is kept however long it took: it takes over when it arrives
        Plan plan;
        plan.valid = converged;
        plan.t0 = t0;
        if (converged) {
            plan.times = nmpc_.times;
            plan.inputs = nmpc_.plan_inputs;
            plan.path = nmpc_.plan;
        } else {
            char why[96];
            std::snprintf(why, sizeof why, "NMPC did not converge (%.0f ms)", 1e3 * took);
            plan.reason = why;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            solving_ = false;
            ++status_.solves;
            status_.solves_on_time += converged && took <= options_.period;
            status_.last_solve = took;
            status_.last_converged = converged;
            next_ = std::move(plan);
            has_next_ = true;
        }
        next += period;
        if (Clock::now() > next) next = Clock::now();  // past its period: the next solve starts at once
        std::this_thread::sleep_until(next);
    }
}

void FallbackControl::pure_pursuit_loop() {
    pthread_setname_np(pthread_self(), "pure_pursuit");  // as top -H shows it
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / options_.rate));
    Clock::time_point next = Clock::now();
    while (running_) {
        double t;
        const State x = bicycle_state.state(&t);
        const Input u = pure_pursuit_driver_(t, x);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pure_pursuit_u_ = u;
            pure_pursuit_valid_ = true;
            ++status_.pure_pursuit_updates;
        }
        next = std::max(next + period, Clock::now());
        std::this_thread::sleep_until(next);
    }
}

void FallbackControl::command_loop() {
    pthread_setname_np(pthread_self(), "command");  // as top -H shows it
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / options_.rate));
    // how far the weight moves in a command
    const double step = options_.blend > 0.0 ? 1.0 / (options_.rate * options_.blend) : 1.0;
    std::string failure;  // why the latest solve gave no plan, until one does
    Clock::time_point next = Clock::now();
    while (running_) {
        next += period;
        std::this_thread::sleep_until(next);
        const double t = bicycle_state.time();
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_next_ && t >= next_.t0) {  // the next plan's (or failure's) time has come
            if (next_.valid) {
                current_ = std::move(next_);
                failure.clear();
            } else {  // the last plan stays, for the NMPC's part of the handover
                failure = next_.reason;
            }
            has_next_ = false;
        }
        // the solve under way should have taken over by now
        const bool late = solving_ && !has_next_ && t >= solving_t0_;
        const double age = t - current_.t0;
        const int k = current_.valid ? tc::step_at(current_.times, age) : -1;
        const bool in_plan = k >= 0 && k < current_.inputs.rows();
        if (in_plan) {
            nmpc_u_ = current_.inputs.row(k).transpose();
            nmpc_valid_ = true;
        }
        const bool nmpc_wanted = in_plan && !late && failure.empty();
        Status& st = status_;
        st.nmpc_u = in_plan ? nmpc_u_ : Input::Constant(NAN);
        st.pure_pursuit_u = pure_pursuit_valid_ ? pure_pursuit_u_ : Input::Constant(NAN);
        double w = st.weight;
        w += std::clamp((nmpc_wanted ? 0.0 : 1.0) - w, -step, step);
        if (!nmpc_valid_) w = 1.0;
        if (!pure_pursuit_valid_) w = 0.0;
        Source source = Source::none;
        Input u = Input::Zero();
        if (nmpc_valid_ || pure_pursuit_valid_) {
            u = (1.0 - w) * nmpc_u_ + w * pure_pursuit_u_;
            source = w <= 0.0 ? Source::nmpc : w >= 1.0 ? Source::pure_pursuit : Source::blend;
            st.weight = w;
        }
        if (in_plan) st.plan_age = age;
        if (nmpc_wanted) {
            st.reason.clear();
        } else {
            char why[96];
            if (late) std::snprintf(why, sizeof why, "NMPC solve past %.0f ms", 1e3 * options_.period);
            st.reason = late ? why : !failure.empty() ? failure : !current_.valid ? current_.reason : "NMPC plan ran out";
        }
        if (source != Source::none) {
            bicycle_state.command(u);
            ++st.ticks;
            ++(source == Source::nmpc ? st.nmpc_ticks : source == Source::pure_pursuit ? st.pure_pursuit_ticks
                                                                                         : st.blend_ticks);
        }
        st.source = source;
        if (Clock::now() - next > 10 * period) next = Clock::now();  // fell far behind (a stall): restart the beat
    }
}

}  // namespace sim
