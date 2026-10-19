#include "sim/racetrack/fallback_control.hpp"

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

// The state `lead` seconds on from x, holding the (already limited) input u.
State predict(const State& x, const Input& u, const VehicleParams& p, double lead) {
    const int steps = std::max(1, static_cast<int>(std::lround(lead / 0.005)));
    State z = x;
    for (int i = 0; i < steps; ++i)
        z = rk4_step([&](const State& s, const Input& v) { return dynamics(s, v, p); }, z, u, lead / steps);
    return z;
}

}  // namespace

const char* source_name(FallbackControl::Source source) {
    switch (source) {
        case FallbackControl::Source::nmpc: return "nmpc";
        case FallbackControl::Source::pure_pursuit: return "pure_pursuit";
        default: return "none";
    }
}

FallbackControl::FallbackControl(Driver& nmpc, Driver& pure_pursuit, Options options)
    : nmpc_driver_(nmpc), pure_pursuit_driver_(pure_pursuit), nmpc_(nmpc_of(nmpc)), options_(options) {
    if (!(options.rate > 0.0) || !(options.deadline > 0.0))
        throw std::invalid_argument("FallbackControl: rate and deadline must be positive");
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
    while (running_) {
        double t;
        const State x = bicycle_state.state(&t);
        const Input u = bicycle_state.applied_input();
        const double t0 = t + options_.deadline;  // the plan starts where the solve must have ended
        const State x0 = predict(x, u, nmpc_.p, options_.deadline);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            solving_ = true;
            solving_t0_ = t0;
        }
        const Clock::time_point start = Clock::now();
        nmpc_driver_(t0, x0);
        const double took = std::chrono::duration<double>(Clock::now() - start).count();
        const bool converged = nmpc_.last_success;
        Plan plan;
        plan.valid = converged && took <= options_.deadline;
        if (plan.valid) {
            plan.t0 = t0;
            plan.h = nmpc_.h;
            plan.inputs = nmpc_.plan_inputs;
            plan.path = nmpc_.plan;
        } else {
            char why[96];
            if (!converged) std::snprintf(why, sizeof why, "NMPC did not converge (%.0f ms)", 1e3 * took);
            else std::snprintf(why, sizeof why, "NMPC took %.0f ms > %.0f ms", 1e3 * took, 1e3 * options_.deadline);
            plan.reason = why;
            plan.t0 = t0;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        solving_ = false;
        ++status_.solves;
        status_.solves_on_time += plan.valid;
        status_.last_solve = took;
        status_.last_converged = converged;
        next_ = std::move(plan);
        has_next_ = true;
    }
}

void FallbackControl::pure_pursuit_loop() {
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
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / options_.rate));
    Clock::time_point next = Clock::now();
    while (running_) {
        next += period;
        std::this_thread::sleep_until(next);
        const double t = bicycle_state.time();
        Input u;
        Source source = Source::none;
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_next_ && t >= next_.t0) {  // the next plan's (or failure's) time has come
            current_ = std::move(next_);
            has_next_ = false;
        }
        // the solve under way should have taken over by now
        const bool late = solving_ && !has_next_ && t >= solving_t0_;
        const double age = t - current_.t0;
        const int k = static_cast<int>(std::floor(age / current_.h));
        if (current_.valid && !late && k >= 0 && k < current_.inputs.rows()) {
            u = current_.inputs.row(k).transpose();
            source = Source::nmpc;
            status_.plan_age = age;
            status_.reason.clear();
        } else if (pure_pursuit_valid_) {
            u = pure_pursuit_u_;
            source = Source::pure_pursuit;
            char why[96];
            if (late) std::snprintf(why, sizeof why, "NMPC solve past %.0f ms", 1e3 * options_.deadline);
            status_.reason = late ? why : !current_.valid ? current_.reason : "NMPC plan ran out";
        }
        if (source != Source::none) {
            bicycle_state.command(u);
            ++status_.ticks;
            ++(source == Source::nmpc ? status_.nmpc_ticks : status_.pure_pursuit_ticks);
        }
        status_.source = source;
        if (Clock::now() - next > 10 * period) next = Clock::now();  // fell far behind (a stall): restart the beat
    }
}

}  // namespace sim
