// The race track: two lanes of asphalt, a grass strip on each side, slower traffic.
//
// Read from a JSON file (config/racetrack.json). The centreline is the lane
// divider and is the path the controllers follow; the right lane (e_y < 0) is
// the ego's home lane. Traffic cars have no dynamics: each holds a lane centre
// at constant speed along that lane.
#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

#include "track_control/controller.hpp"
#include "track_control/path.hpp"

namespace sim {

using namespace tc;

using Corners = Eigen::Matrix<double, 4, 2>;  // corners of a car body, one per row

// Constant-speed car on a lane centre. `at` is a world point near its position at t = 0.
struct TrafficCar {
    std::array<double, 2> at;
    std::string lane = "right";
    double speed = 50.0 / 3.6;  // [m/s]
};

struct TrafficPose {
    double x, y, psi, s;  // body centre, heading, arc length of the car's lane point
};

struct Circuit {
    // Read `file` (config/racetrack.json); the default is the one built into this repository.
    explicit Circuit(const std::string& file = default_file());
    static std::string default_file();

    Path track;
    double lane_width = 3.5;  // [m]
    double grass_width = 1.75;  // [m]
    double mu_road = 0.9;  // asphalt
    double mu_grass = 0.3;
    double v_max = 80.0 / 3.6;  // ego speed limit [m/s]
    std::string ego_lane = "right";
    double ego_start_speed = 50.0 / 3.6;  // [m/s]
    Body body;  // every car's body: CARLA's Model 3 bounding box
    std::vector<TrafficCar> traffic;

    double road_half_width() const { return lane_width; }
    double half_width() const { return lane_width + grass_width; }  // centreline to the outer edge of the grass
    double lane_offset(const std::string& lane) const;  // e_y of a lane centre [m]
    double friction(double e_y) const;  // friction coefficient at lateral offset e_y; nan beyond the grass
    Road road() const;  // the asphalt, as the controllers see it

    double traffic_s(const TrafficCar& car, double t) const;  // arc length of a traffic car at time t, 0..lap
    TrafficPose traffic_pose(const TrafficCar& car, double t) const;  // body centre and heading at time t, and s

    State initial_state() const;  // on the start line in the ego lane at the start speed
    PathState track_state(const State& x) const;
    Corners ego_corners(const State& x) const;
    Corners body_corners(double x, double y, double psi) const;  // a car body centred on (x, y)
    double clearance(double t, const State& x) const;  // ego body to the nearest traffic body [m], 0 on overlap
    double road_margin(const State& x) const;  // ego body to the nearer asphalt edge [m], negative on the grass
};

// Distance between two convex polygons, 0 if they overlap.
double polygon_distance(const Corners& P, const Corners& Q);

}  // namespace sim
