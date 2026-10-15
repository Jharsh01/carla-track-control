// Closed-loop run of a controller round the circuit, offline (no CARLA).
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "sim/plant/interface.hpp"
#include "sim/racetrack/circuit.hpp"

namespace sim {

using ControlLaw = std::function<Input(double t, const State& x)>;

struct LapResult {
    bool completed;
    std::optional<double> lap_time;  // [s]
    bool collided;
    bool left_track;  // centre of gravity went beyond the grass
    double min_clearance;  // smallest distance to a traffic car body [m]
    double road_margin;  // smallest distance from the ego body to the asphalt edge [m], negative on the grass
    double time_on_grass;  // time with the centre of gravity on the grass [s]
    double peak_sideslip;  // [rad]
    double min_speed;  // [m/s]
    double max_speed;  // [m/s]
};

// One row per logged sample; s counts up from 0 without wrapping.
struct LapLog {
    std::vector<double> t, s, e_y, mu;
    std::vector<State> x;
    std::vector<Input> u;
};

// Drive `plant` from the start line round `circuit` with `controller(t, x) -> u`.
//
// The controller is called every `dt_ctrl` seconds and its input held in
// between; the state is logged every `dt_log` seconds. The plant's friction
// follows the surface under the centre of gravity (asphalt or grass), and the
// plant is left with that of the last surface. The run ends when the lap is
// complete, at a collision with a traffic car, when the car leaves the grass,
// or at `t_max`.
std::pair<LapLog, LapResult> run_lap(const Circuit& circuit, const ControlLaw& controller, PlantModel& plant,
                                     const State* x0 = nullptr, double t_max = 180.0, double dt_ctrl = 0.05,
                                     double dt_log = 0.01);

// Number of traffic cars the ego got ahead of during the run.
int cars_passed(const Circuit& circuit, const LapLog& log);

}  // namespace sim
