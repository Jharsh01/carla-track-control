// What every controller in the library takes and returns.
//
// A controller follows a `Path` within a `Road`, and is called once per
// control period with an `Observation`: the time, the car's state (as measured,
// noise and all) and the other cars it should keep clear of. It returns the
// input [steering rate, Fx] to hold until the next call (types.hpp), within the
// vehicle parameters' steering-rate and force limits.
//
// Integrating with a simulator means converting its state to `State`, its
// detected vehicles to `Obstacle`s on the path, and the returned input to its
// actuator commands; carla_bridge/ does that for CARLA.
#pragma once

#include <limits>
#include <string>
#include <vector>

#include "track_control/path.hpp"
#include "track_control/types.hpp"
#include "track_control/vehicle_params.hpp"

namespace tc {

constexpr double INF = std::numeric_limits<double>::infinity();

// Another car, in path coordinates.
struct Obstacle {
    double s;  // arc length along the path [m]
    double e_y;  // lateral offset from the path [m]
    double speed;  // its speed along the path [m/s]
};

struct Observation {
    double t = 0.0;  // [s]
    State x = State::Zero();  // the ego car's state
    std::vector<Obstacle> obstacles;
};

// The road around the path. The path is taken to run along a lane boundary or
// the middle of the road; lanes are `lane_width` wide, and the drivable surface
// ends `half_width` either side of the path.
struct Road {
    double lane_width = 3.5;  // [m]
    double half_width = 3.5;  // path to the edge of the drivable surface [m]
    double mu = 0.9;  // tire-road friction the plans may use
    double v_max = 80.0 / 3.6;  // speed limit [m/s]
};

// Size of the ego car's body, for the road-edge and clearance constraints.
struct Body {
    double length = 4.79;  // [m]
    double width = 2.16;  // [m]
    double cg_to_center = 0.45;  // centre of gravity ahead of the body centre [m]
};

class Controller {
public:
    virtual ~Controller() = default;
    // Input to hold from time obs.t on.
    virtual Input compute(const Observation& obs) = 0;
    virtual std::string name() const = 0;
};

// Heading of the car relative to the path, and its path coordinates.
struct PathState {
    double s, e_y, e_psi;
};
PathState path_state(const Path& path, const State& x);

}  // namespace tc
