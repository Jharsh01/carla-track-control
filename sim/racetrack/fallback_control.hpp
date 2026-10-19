// The NMPC with pure pursuit as its fallback, both running at once against the
// real-time bicycle plant (sim/plant/bicycle_plant.hpp), each on its own thread.
//
//   NMPC thread          solves back to back, each solve allowed `deadline`
//                        seconds of wall time. A solve started at plant time t
//                        plans from t + deadline: from the state the car will
//                        have then, predicted with the bicycle model holding the
//                        input being applied. So a solve that makes the deadline
//                        delivers its plan before the plan begins.
//   pure pursuit thread  every period (1 / rate), from the plant's latest state
//   command thread       every period, commands the plant (bicycle_state) with
//                        - the NMPC plan whose start has come, played back at the
//                          plant's time (the input it planned for that moment),
//                          if its solve converged within the deadline;
//                        - pure pursuit's latest input otherwise: when that solve
//                          did not converge or ran past the deadline, or the solve
//                          under way has not delivered by the time its plan should
//                          begin.
//
// So an input is commanded every period whatever the NMPC does, and `status`
// says which controller's it is, and why pure pursuit's when it is.
#pragma once

#include <atomic>
#include <chrono>
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
        double rate = 100.0;  // command and pure pursuit [Hz]
        double deadline = 0.1;  // wall time an NMPC solve may take [s]
    };

    enum class Source { none, nmpc, pure_pursuit };

    struct Status {
        Source source = Source::none;  // whose input was commanded last
        std::string reason;  // why pure pursuit's, when it was
        double plan_age = 0.0;  // plant time since the observation the NMPC plan in use started from [s]
        double last_solve = 0.0;  // wall time of the last NMPC solve [s]
        bool last_converged = false;
        uint64_t ticks = 0, nmpc_ticks = 0, pure_pursuit_ticks = 0;  // commands, and whose
        uint64_t solves = 0, solves_on_time = 0;  // NMPC solves, and those converged within the deadline
        uint64_t pure_pursuit_updates = 0;
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
        double t0 = 0.0, h = 0.1;
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
    Status status_;
};

const char* source_name(FallbackControl::Source source);

}  // namespace sim
