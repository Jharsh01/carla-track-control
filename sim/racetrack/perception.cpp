#include "sim/racetrack/perception.hpp"

#include <algorithm>

namespace sim {

const State EGO_SIGMA = (State() << 0.1, 0.1, 0.005, 0.1, 0.05, 0.005, 0.002).finished();

static std::mt19937_64 seeded(std::initializer_list<unsigned> words) {
    std::seed_seq sequence(words);
    return std::mt19937_64(sequence);
}

Perception::Perception(const Circuit& circuit, double range_ahead, double range_behind, double sigma_s,
                       double sigma_e, double sigma_v, unsigned seed, std::optional<State> sigma_ego,
                       double road_range)
    : c(circuit),
      range_ahead(range_ahead),
      range_behind(range_behind),
      sigma{sigma_s, sigma_e, sigma_v},
      sigma_ego(std::move(sigma_ego)),
      road_range(road_range),
      rng_(seed),
      rng_ego_(seeded({seed, 1u})) {}

Perception Perception::limited(const Circuit& circuit, unsigned seed) {
    return Perception(circuit, 60.0, 20.0, 0.5, 0.2, 0.5, seed, EGO_SIGMA, 40.0);
}

std::vector<Obstacle> Perception::observe(double t, double s_ego) {
    std::vector<Obstacle> out;
    for (const TrafficCar& car : c.traffic) {
        const double s = c.traffic_s(car, t);
        const double ahead = c.track.difference(s, s_ego);
        if (!(-range_behind <= ahead && ahead <= range_ahead)) continue;
        const double ds = normal_(rng_) * sigma[0], de = normal_(rng_) * sigma[1], dv = normal_(rng_) * sigma[2];
        out.push_back({c.track.wrap(s + ds), c.lane_offset(car.lane) + de, car.speed + dv});
    }
    return out;
}

State Perception::ego(const State& x) {
    if (!sigma_ego) return x;
    State out = x;
    for (int i = 0; i < NX; ++i) out[i] += normal_ego_(rng_ego_) * (*sigma_ego)[i];
    return out;
}

}  // namespace sim
