// Pure pursuit with a grip-limited speed profile and a lane-change rule for passing.
//
// Steering is the pure-pursuit law on a line offset from the path: the
// lookahead point is on the current target line, L_d = max(k_lookahead * U_x,
// lookahead_min) further along the path than the rear axle. Speed follows a
// grip-limited profile (speed_profile.hpp): computed once for the whole path if
// the road is known everywhere (`road_range` infinite), otherwise at every call
// over the `road_range` metres ahead. The longitudinal force is the profile's
// acceleration plus a proportional speed correction plus drag. Both outputs are
// kept within the vehicle's limits (steering rate, model/bicycle.hpp fx_limits).
//
// Pure pursuit has no notion of other cars, so passing is a rule: move from
// `lane_offset` to `pass_offset` while an obstacle near `lane_offset` is
// between `window_behind` behind and `window_ahead` ahead of the ego (centre to
// centre, along the path), and come back afterwards. The window ahead grows with
// the closing speed, so that `pass_time` seconds are left to change lane before
// reaching the car. The target line moves at `lateral_rate` [m/s].
//
// With a limited road range the car keeps to a speed from which it can brake,
// within the known road, to the speed of the path's tightest corner: the
// tightest radius is taken as known in advance (a property of the road).
#pragma once

#include <optional>
#include <vector>

#include "track_control/controller.hpp"

namespace tc {

class PurePursuit : public Controller {
public:
    struct Options {
        double lane_offset = 0.0;  // line followed when nothing is in the way [m]
        double pass_offset = 3.5;  // line to pass on [m]
        double road_range = INF;  // how far ahead the road is known [m]
        double grip_use = 0.4;  // share of mu*g the speed profile may use
        double a_accel = 3.0;  // [m/s^2]
        double brake_use = 0.85;  // share of the brakes' peak force the profile plans with
        double k_lookahead = 0.6;  // [s]
        double lookahead_min = 5.0;  // [m]
        double tau_steer = 0.05;  // time constant the steering rate closes the angle error with [s]
        double k_speed = 3.0;  // [1/s]
        double window_ahead = 30.0;  // [m]
        double window_behind = 15.0;  // [m]
        double pass_time = 3.5;  // [s]
        double lateral_rate = 1.2;  // [m/s]
    };

    PurePursuit(Path path, Road road, VehicleParams params, Body body, Options options);
    PurePursuit(Path path, Road road, VehicleParams params)
        : PurePursuit(std::move(path), road, params, Body(), Options()) {}

    Input compute(const Observation& obs) override;
    std::string name() const override { return "pure_pursuit"; }

    // Profile speed [m/s] and its slope dv/ds [1/s] at arc length `s`.
    std::pair<double, double> speed_reference(double s) const;
    double target_offset() const { return e_target_; }  // the line it is steering onto now [m]

    const Path path;
    const Road road;
    const VehicleParams p;
    const Body body;
    const Options opt;

private:
    bool obstacle_in_the_way(const std::vector<Obstacle>& obstacles, double s, double speed) const;

    double e_target_;
    double a_grip_, a_brake_;
    bool limited_;  // the road is known only a limited distance ahead
    std::vector<double> k_line_, v_limit_;  // cornering limit along the path (limited road range)
    std::vector<double> s_grid_, v_grid_;  // speed profile of the path (whole road known)
    std::optional<double> t_prev_;
};

}  // namespace tc
