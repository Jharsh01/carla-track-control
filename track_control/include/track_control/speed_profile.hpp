// Grip-limited speed profile along a path.
#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "track_control/path.hpp"

namespace tc {

struct CorneringLimit {
    std::vector<double> s;  // grid of arc lengths along the path
    std::vector<double> k_line;  // curvature of the tightest of the offset lines at each point
    std::vector<double> v;  // the speed min(v_max, sqrt(a / |kappa|)) it allows
};

// Grid of arc lengths along the path (spacing about `ds`), with the curvature of
// the tightest of the lines at the lateral `offsets` and the speed it allows.
CorneringLimit cornering_limit(const Path& path, const std::vector<double>& offsets, double a, double v_max,
                               double ds = 0.5);

// Speed at the first two points of a stretch of known road, given what lies beyond it is unknown.
//
// `v_limit` and `k_line` are the cornering limit and curvature at points `step`
// apart, from the car to the end of the known road. The car must be able to
// brake to each point's limit, and to `v_beyond` at the last one: the speed
// that is safe for anything the road may do next. Braking shares the budget `a`
// with cornering and is capped at `a_brake`.
std::pair<double, double> speed_within_sight(std::vector<double> v_limit, const std::vector<double>& k_line,
                                             double step, double a, double a_brake, double v_beyond);

// Speed [m/s] against arc length for lines at the lateral `offsets` from the path.
//
// At each point the speed is the lowest of `v_max`, the cornering limit
// sqrt(a / |kappa|) of the tightest of the offset lines, what braking can still
// shed before the next corner and what accelerating has reached since the last
// one. a = grip_use * mu * g is the acceleration budget; braking and
// accelerating share it with cornering (friction circle), and are also capped
// at `a_brake` and `a_accel` (what the brakes and the drive can deliver). On a
// closed path the passes carry round the lap; an open path ends at `v_max`.
//
// Returns (s, v) on a grid of spacing about `ds` covering the path.
std::pair<std::vector<double>, std::vector<double>> grip_speed_profile(
    const Path& path, const std::vector<double>& offsets, double mu, double v_max, double grip_use = 0.8,
    double a_accel = 3.0, double a_brake = std::numeric_limits<double>::infinity(), double ds = 0.5,
    double g = 9.81);

}  // namespace tc
