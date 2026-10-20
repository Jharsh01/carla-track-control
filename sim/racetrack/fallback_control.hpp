// The NMPC with pure pursuit as its fallback, both running at once against the
// real-time bicycle plant (sim/plant/bicycle_plant.hpp), each on its own thread.
//
//   NMPC thread          solves every `period` seconds (200 ms). A solve started
//                        at plant time t plans from t + period: from the state
//                        the car will have then, predicted with the bicycle model
//                        and the inputs the current plan sends meanwhile. Each
//                        plan is played for one period, `rate` x `period`
//                        commands (10 at 50 Hz), until the next takes over. A
//                        solve that runs past its period is not dropped: its plan
//                        takes over when it arrives, from the input it planned for
//                        that moment, and the next solve starts at once.
//   pure pursuit thread  every 1 / rate, from the plant's latest state
//   command thread       every 1 / rate, commands the plant (bicycle_state) with a
//                        blend of the two, (1 - w) u_nmpc + w u_pure_pursuit, the
//                        weight w moving to 0 while the NMPC has a plan for the
//                        moment and to 1 while it has none (its solve is past its
//                        period, did not converge, or its plan ran out), across
//                        the whole range in `blend` seconds, so a handover either
//                        way is a ramp rather than a step. While handing over to
//                        pure pursuit the NMPC's part is its last plan carried on
//                        past its period (it covers the whole horizon), or its
//                        last input once that runs out.
//
// So an input is commanded every period whatever the NMPC does, and `status`
// says whose it is (or that it is a blend), and why pure pursuit's when it is.
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "sim/racetrack/driver.hpp"
#include "track_control/nmpc.hpp"

namespace sim {

class FallbackControl {
public:
    struct Options {
        double rate = 50.0;  // command and pure pursuit [Hz]
        double period = 0.2;  // a solve every period; the plan it makes starts a period after the solve does [s]
        double blend = 0.2;  // time a handover from one controller to the other takes [s]; 0: a switch
    };

    enum class Source { none, nmpc, pure_pursuit, blend };

    struct Status {
        Source source = Source::none;  // whose input was commanded last
        std::string reason;  // why pure pursuit's, when it was
        double plan_age = 0.0;  // plant time since the start of the NMPC plan in use [s]
        double last_solve = 0.0;  // wall time of the last NMPC solve [s]
        bool last_converged = false;
        double weight = 1.0;  // pure pursuit's share of the last command: 0 the NMPC's alone, 1 pure pursuit's alone
        uint64_t ticks = 0, nmpc_ticks = 0, pure_pursuit_ticks = 0, blend_ticks = 0;  // commands, and whose
        uint64_t solves = 0, solves_on_time = 0;  // NMPC solves, and those converged within the period
        uint64_t pure_pursuit_updates = 0;
        // What each would command now, whoever is in command: the NMPC's input from its latest converged plan,
        // pure pursuit's latest; NaN while there is none.
        Input nmpc_u = Input::Constant(NAN), pure_pursuit_u = Input::Constant(NAN);
    };

    // Both drivers must outlive this and are used only by its threads while it runs.
    FallbackControl(Driver& nmpc, Driver& pure_pursuit, Options options);
    FallbackControl(Driver& nmpc, Driver& pure_pursuit) : FallbackControl(nmpc, pure_pursuit, Options()) {}
    ~FallbackControl() { stop(); }

    // Start the threads; the plant must be running. Until the first solve or pure pursuit input,
    // the plant keeps the input commanded before.
    void start();
    void stop();
    Status status() const;
    std::vector<std::pair<double, double>> plan() const;  // (s, e_y) of the NMPC plan in use; empty if none

private:
    using Clock = std::chrono::steady_clock;
    void nmpc_loop();
    void pure_pursuit_loop();
    void command_loop();

    Driver &nmpc_driver_, &pure_pursuit_driver_;
    tc::NMPC& nmpc_;
    Options options_;
    std::atomic<bool> running_{false};
    std::vector<std::thread> threads_;

    // An NMPC plan from start time t0 (plant time), or why there is none (`valid` false).
    struct Plan {
        bool valid = false;
        std::string reason = "no NMPC solve yet";
        double t0 = 0.0;
        std::vector<double> times;  // when each step starts, from t0, as NMPC::times
        Eigen::MatrixXd inputs, path;  // as NMPC::plan_inputs and NMPC::plan
    };

    mutable std::mutex mutex_;  // guards everything below
    Plan current_;  // in use (or the reason for pure pursuit)
    Plan next_;  // delivered, to take over at its t0
    bool has_next_ = false;
    bool solving_ = false;
    double solving_t0_ = 0.0;  // the start of the plan the solve under way is making
    // pure pursuit's latest input
    bool pure_pursuit_valid_ = false;
    Input pure_pursuit_u_ = Input::Zero();
    // the NMPC's last input, its part of a blend once its plan has run out
    bool nmpc_valid_ = false;
    Input nmpc_u_ = Input::Zero();
    Status status_;
};

const char* source_name(FallbackControl::Source source);

}  // namespace sim
