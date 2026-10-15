// What a controller is told: noisy traffic detections, a noisy ego state, the road a limited distance ahead.
#pragma once

#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "sim/racetrack/circuit.hpp"

namespace sim {

// standard deviations of the ego state errors [X, Y, psi, Ux, Uy, r, delta] used by `Perception::limited`:
// m, m, rad, m/s, m/s, rad/s, rad
extern const State EGO_SIGMA;

// Traffic, the ego's own state and the road as known to the ego car.
//
// A car is reported while it is within `range_ahead` in front of the ego or
// `range_behind` behind it, measured along the track. Each report carries
// independent Gaussian errors of standard deviation `sigma_s` [m] along the
// track, `sigma_e` [m] across it and `sigma_v` [m/s] in speed, drawn afresh at
// every call.
//
// The ego state is reported with independent Gaussian errors of standard
// deviation `sigma_ego` (one per state), also drawn afresh at every call.
//
// The road's shape is known up to `road_range` [m] ahead of the ego; the
// controllers are configured with it (their `road_range` option).
//
// The defaults give exact reports of every car and of the ego state, and the
// whole road.
//
// Not modelled: line of sight (a car round a corner is reported like any other
// within range), missed or false detections, delay, errors that persist from
// one call to the next (bias, drift).
class Perception {
public:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    explicit Perception(const Circuit& circuit, double range_ahead = INF, double range_behind = INF,
                        double sigma_s = 0.0, double sigma_e = 0.0, double sigma_v = 0.0, unsigned seed = 0,
                        std::optional<State> sigma_ego = std::nullopt, double road_range = INF);

    // A plausible sensor set: traffic 60 m ahead and 20 m behind with 0.5 m / 0.2 m / 0.5 m/s errors,
    // ego state errors `EGO_SIGMA`, the road known 40 m ahead.
    static Perception limited(const Circuit& circuit, unsigned seed = 0);

    // Detections at time `t` for an ego at arc length `s_ego`.
    std::vector<Obstacle> observe(double t, double s_ego);
    // The ego state `x` as measured.
    State ego(const State& x);

    Circuit c;
    double range_ahead, range_behind;
    std::array<double, 3> sigma;
    std::optional<State> sigma_ego;
    double road_range;

private:
    std::mt19937_64 rng_, rng_ego_;  // separate streams, so the ego errors leave the traffic errors as they were
    std::normal_distribution<double> normal_, normal_ego_;
};

}  // namespace sim
