// A library controller set up for the race track, fed through a perception.
//
// This is the glue a simulator needs once per control period: measure the ego
// state, place the detected cars on the track, ask the controller. The offline
// lap (run_lap) and the CARLA bridge both drive through it, so they run the
// same controllers with the same settings.
#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "sim/racetrack/perception.hpp"
#include "track_control/controller.hpp"

namespace sim {

// Margins the NMPC plans with, below what the vehicle parameters say: `mu_use`
// scales the friction of its model, `brake_use` and `drive_use` the peak brake
// and drive force it may ask for, and `sideslip_max` [rad] is its (soft) limit
// on the body sideslip angle. The defaults plan with the parameters as they
// are. (Pure pursuit has its own margins in its speed profile:
// PurePursuit::Options::grip_use, brake_use.)
struct Tuning {
    double mu_use = 1.0;
    double brake_use = 1.0;
    double drive_use = 1.0;
    double sideslip_max = 7.0 * M_PI / 180.0;
    double solve_time_limit = 0.0;  // [s] wall time an NMPC solve may take before it gives up; 0: no limit
    double nmpc_dt = 0.0;  // [s] the NMPC's step; 0: its default (NMPC::Options::h)
    double nmpc_horizon = 0.0;  // [s] how far the NMPC plans; 0: its default (NMPC::Options::horizon)
    double nmpc_fine_dt = 0.0, nmpc_fine = 0.0;  // [s] a finer step for the first nmpc_fine seconds; 0: none
};

// The NMPC margins for a vehicle dynamics ("bicycle", "chrono", "carla") from
// config/tuning.json (or `file`); the defaults if the file has no entry for it.
Tuning load_tuning(const std::string& dynamics, const std::string& file = SIM_CONFIG_DIR "/tuning.json");

// "nmpc" or "pure_pursuit", following the circuit in its ego lane, passing on
// the other one, knowing the road `road_range` metres ahead.
std::unique_ptr<Controller> make_controller(const std::string& name, const Circuit& circuit,
                                            const VehicleParams& params, double road_range,
                                            const Tuning& tuning = Tuning());

class Driver {
public:
    Driver(std::unique_ptr<Controller> controller, Perception perception);
    // The controller's input for the true state `x` at time `t`, after the perception.
    Input operator()(double t, const State& x);

    Controller& controller() { return *controller_; }
    Perception perception;
    Observation last;  // what the controller was given at the last call

private:
    std::unique_ptr<Controller> controller_;
};

}  // namespace sim
