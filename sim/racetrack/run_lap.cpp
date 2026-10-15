#include "sim/racetrack/run_lap.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

std::pair<LapLog, LapResult> run_lap(const Circuit& circuit, const ControlLaw& controller, PlantModel& plant,
                                     const State* x0, double t_max, double dt_ctrl, double dt_log) {
    const Path& track = circuit.track;
    const int substeps = static_cast<int>(std::lround(dt_ctrl / dt_log));
    plant.reset(x0 ? *x0 : circuit.initial_state());
    double t = 0.0;
    State x = plant.observe();
    double s_prev = track.project(x[IX], x[IY]).first;
    double progress = 0.0;
    LapLog log;
    double min_clearance = INFINITY, min_margin = INFINITY;
    std::optional<double> lap_time;
    bool collided = false, left_track = false;

    while (!lap_time && !collided && !left_track && t < t_max) {
        const double clearance = circuit.clearance(t, x);
        min_clearance = std::min(min_clearance, clearance);
        min_margin = std::min(min_margin, circuit.road_margin(x));
        if (clearance <= 0.0) {
            collided = true;
            break;
        }
        const Input u = plant.saturate(controller(t, x));
        for (int i = 0; i < substeps; ++i) {
            const auto [s, e_y] = track.project(x[IX], x[IY]);
            progress += track.difference(s, s_prev);
            s_prev = s;
            const double mu = circuit.friction(e_y);
            log.t.push_back(t);
            log.x.push_back(x);
            log.u.push_back(u);
            log.s.push_back(progress);
            log.e_y.push_back(e_y);
            log.mu.push_back(mu);
            if (std::isnan(mu)) {
                left_track = true;
                break;
            }
            if (progress >= track.length()) {
                lap_time = t;
                break;
            }
            plant.set_friction(mu);
            x = plant.step(u, dt_log);
            t += dt_log;
        }
    }

    LapResult result{};
    result.completed = lap_time.has_value();
    result.lap_time = lap_time;
    result.collided = collided;
    result.left_track = left_track;
    result.min_clearance = min_clearance;
    result.road_margin = min_margin;
    result.time_on_grass = dt_log * std::count(log.mu.begin(), log.mu.end(), circuit.mu_grass);
    result.peak_sideslip = 0.0;
    result.min_speed = INFINITY;
    result.max_speed = -INFINITY;
    for (const State& xk : log.x) {
        result.peak_sideslip = std::max(result.peak_sideslip, std::fabs(std::atan2(xk[IUY], xk[IUX])));
        result.min_speed = std::min(result.min_speed, xk[IUX]);
        result.max_speed = std::max(result.max_speed, xk[IUX]);
    }
    return {std::move(log), result};
}

int cars_passed(const Circuit& circuit, const LapLog& log) {
    const double lap = circuit.track.length();
    int count = 0;
    for (const TrafficCar& car : circuit.traffic) {
        // every car starts somewhere ahead on the lap
        const double ahead = pymod(circuit.traffic_s(car, log.t.front()) - log.s.front(), lap);
        const double car_travel = car.speed * (log.t.back() - log.t.front());
        count += log.s.back() - log.s.front() > ahead + car_travel;
    }
    return count;
}

}  // namespace sim
