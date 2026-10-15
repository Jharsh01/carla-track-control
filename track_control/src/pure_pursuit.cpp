#include "track_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

#include "track_control/model/bicycle.hpp"
#include "track_control/speed_profile.hpp"

namespace tc {

// Linear interpolation of the samples (xp, fp); periodic with `period` on a lap (as numpy.interp with `period`).
static double interp_path(double x, const std::vector<double>& xp, const std::vector<double>& fp, double period,
                          bool periodic) {
    const size_t n = xp.size();
    if (!periodic) {
        if (x <= xp.front()) return fp.front();
        if (x >= xp.back()) return fp.back();
    }
    x = periodic ? pymod(x, period) : x;
    const size_t i = std::upper_bound(xp.begin(), xp.end(), x) - xp.begin();  // xp[i - 1] <= x < xp[i]
    const size_t lo = (i + n - 1) % n, hi = i % n;
    const double x_lo = i == 0 ? xp[lo] - period : xp[lo];
    const double x_hi = i == n ? xp[hi] + period : xp[hi];
    return fp[lo] + (x - x_lo) / (x_hi - x_lo) * (fp[hi] - fp[lo]);
}

PurePursuit::PurePursuit(Path path_, Road road_, VehicleParams params, Body body_, Options options)
    : path(std::move(path_)),
      road(road_),
      p(params),
      body(body_),
      opt(options),
      e_target_(options.lane_offset),
      a_grip_(options.grip_use * road_.mu * params.g),
      a_brake_(options.brake_use * params.F_brake_max / params.m),
      limited_(std::isfinite(options.road_range)) {
    // the profile must hold on whichever line the car is on
    const std::vector<double> offsets = {opt.lane_offset, opt.pass_offset};
    if (limited_) {
        CorneringLimit limit = cornering_limit(path, offsets, a_grip_, road.v_max);
        k_line_ = std::move(limit.k_line);
        v_limit_ = std::move(limit.v);
    } else {
        std::tie(s_grid_, v_grid_) =
            grip_speed_profile(path, offsets, road.mu, road.v_max, opt.grip_use, opt.a_accel, a_brake_, 0.5, p.g);
    }
}

std::pair<double, double> PurePursuit::speed_reference(double s) const {
    const double length = path.length();
    const bool loop = path.closed();
    if (!limited_) {
        const double v = interp_path(s, s_grid_, v_grid_, length, loop);
        return {v, interp_path(s + 1.0, s_grid_, v_grid_, length, loop) - v};
    }
    const int n = static_cast<int>(v_limit_.size());
    const double step = length / n;
    const int first = static_cast<int>(s / step), count = static_cast<int>(opt.road_range / step) + 1;
    std::vector<double> v(count), k(count);
    for (int i = 0; i < count; ++i) {
        const int j = loop ? ((first + i) % n + n) % n : std::clamp(first + i, 0, n - 1);
        v[i] = v_limit_[j];
        k[i] = k_line_[j];
    }
    const double v_beyond = *std::min_element(v_limit_.begin(), v_limit_.end());
    const auto [v0, v1] = speed_within_sight(std::move(v), k, step, a_grip_, a_brake_, v_beyond);
    return {v0, (v1 - v0) / step};
}

bool PurePursuit::obstacle_in_the_way(const std::vector<Obstacle>& obstacles, double s, double speed) const {
    for (const Obstacle& car : obstacles) {
        if (std::fabs(car.e_y - opt.lane_offset) > 0.5 * road.lane_width) continue;  // not on the home line
        const double ahead = path.difference(car.s, s);
        const double reach = body.length + opt.pass_time * std::max(speed - car.speed, 0.0);
        if (-opt.window_behind <= ahead && ahead <= std::max(opt.window_ahead, reach)) return true;
    }
    return false;
}

Input PurePursuit::compute(const Observation& obs) {
    const State& x = obs.x;
    const double t = obs.t;
    const double s = path.project(x[IX], x[IY]).first;

    // passing rule: slide the target line toward the one it should be on
    const double goal = obstacle_in_the_way(obs.obstacles, s, x[IUX]) ? opt.pass_offset : opt.lane_offset;
    const double dt = t_prev_ ? t - *t_prev_ : 0.0;
    t_prev_ = t;
    e_target_ += std::clamp(goal - e_target_, -opt.lateral_rate * dt, opt.lateral_rate * dt);

    const double psi = x[IPSI];
    const double rear_x = x[IX] - p.b * std::cos(psi), rear_y = x[IY] - p.b * std::sin(psi);
    const double L_d = std::min(std::max(opt.k_lookahead * x[IUX], opt.lookahead_min), opt.road_range);
    const Pose target = path.pose(path.project(rear_x, rear_y).first + L_d, e_target_);
    const double alpha = std::atan2(target.y - rear_y, target.x - rear_x) - psi;
    const double chord = std::hypot(target.x - rear_x, target.y - rear_y);
    const double delta_cmd =
        std::clamp(std::atan(2.0 * p.L() * std::sin(alpha) / chord), -p.delta_max, p.delta_max);
    const double rate = std::clamp((delta_cmd - x[IDELTA]) / opt.tau_steer, -p.delta_rate_max, p.delta_rate_max);

    const auto [v_ref, dv_ds] = speed_reference(s);
    double accel = v_ref * dv_ds + opt.k_speed * (v_ref - x[IUX]);
    if (limited_) accel = std::min(accel, opt.a_accel);  // the full profile has the drive's limit built in; this one has not
    const double drag = 0.5 * p.rho * p.CdA * x[IUX] * x[IUX] + p.Crr * p.m * p.g;
    const auto [fx_min, fx_max] = fx_limits(x[IUX], p);  // what the car can deliver, as the NMPC is bounded
    return {rate, std::clamp(p.m * accel + drag, fx_min, fx_max)};
}

}  // namespace tc
