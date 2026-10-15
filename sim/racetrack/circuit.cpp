#include "sim/racetrack/circuit.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace sim {

using nlohmann::json;

static json read(const std::string& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("cannot open " + file);
    return json::parse(in);
}

static Point point(const json& value) { return {value.at(0).get<double>(), value.at(1).get<double>()}; }

static Path read_track(const std::string& file) {
    const json data = read(file);
    std::vector<Point> corners;
    for (const json& corner : data.at("corners")) corners.push_back(point(corner));
    return Path::from_corners(corners, data.at("radii").get<std::vector<double>>(), point(data.at("start")));
}

std::string Circuit::default_file() { return SIM_CONFIG_DIR "/racetrack.json"; }

Circuit::Circuit(const std::string& file) : track(read_track(file)) {
    const json data = read(file);
    lane_width = data.value("lane_width", lane_width);
    grass_width = data.value("grass_width", grass_width);
    mu_road = data.value("mu_road", mu_road);
    mu_grass = data.value("mu_grass", mu_grass);
    if (data.contains("speed_limit_kmh")) v_max = data.at("speed_limit_kmh").get<double>() / 3.6;
    if (data.contains("ego")) {
        const json& ego = data.at("ego");
        ego_lane = ego.value("lane", ego_lane);
        if (ego.contains("start_speed_kmh")) ego_start_speed = ego.at("start_speed_kmh").get<double>() / 3.6;
    }
    for (const json& car : data.value("traffic", json::array())) {
        TrafficCar t{point(car.at("at"))};
        t.lane = car.value("lane", t.lane);
        if (car.contains("speed_kmh")) t.speed = car.at("speed_kmh").get<double>() / 3.6;
        lane_offset(t.lane);  // checks the name
        traffic.push_back(t);
    }
    lane_offset(ego_lane);
}

double Circuit::lane_offset(const std::string& lane) const {
    if (lane == "left") return 0.5 * lane_width;
    if (lane == "right") return -0.5 * lane_width;
    throw std::invalid_argument("lane must be 'left' or 'right', not '" + lane + "'");
}

double Circuit::friction(double e_y) const {
    const double e = std::fabs(e_y);
    if (e <= road_half_width()) return mu_road;
    if (e <= half_width()) return mu_grass;
    return std::numeric_limits<double>::quiet_NaN();
}

Road Circuit::road() const {
    Road r;
    r.lane_width = lane_width;
    r.half_width = road_half_width();
    r.mu = mu_road;
    r.v_max = v_max;
    return r;
}

double Circuit::traffic_s(const TrafficCar& car, double t) const {
    const double e_y = lane_offset(car.lane);
    // distance along its lane from the start line to where the car is at t = 0
    const double start = track.offset_length(track.project(car.at[0], car.at[1]).first, e_y);
    return track.offset_to_s(start + car.speed * t, e_y);
}

TrafficPose Circuit::traffic_pose(const TrafficCar& car, double t) const {
    const double s = traffic_s(car, t);
    const Pose p = track.pose(s, lane_offset(car.lane));
    return {p.x, p.y, p.psi, s};
}

State Circuit::initial_state() const {
    const Pose p = track.pose(0.0, lane_offset(ego_lane));
    State x = State::Zero();
    x[IX] = p.x;
    x[IY] = p.y;
    x[IPSI] = p.psi;
    x[IUX] = ego_start_speed;
    return x;
}

PathState Circuit::track_state(const State& x) const { return path_state(track, x); }

Corners Circuit::body_corners(double x, double y, double psi) const {
    const double hl = 0.5 * body.length, hw = 0.5 * body.width;
    const double c = std::cos(psi), s = std::sin(psi);
    Corners local;
    local << hl, hw, -hl, hw, -hl, -hw, hl, -hw;
    Eigen::Matrix2d rotation;
    rotation << c, s, -s, c;
    Corners out = local * rotation;
    out.rowwise() += Eigen::RowVector2d(x, y);
    return out;
}

Corners Circuit::ego_corners(const State& x) const {
    const double c = std::cos(x[IPSI]), s = std::sin(x[IPSI]);
    return body_corners(x[IX] - body.cg_to_center * c, x[IY] - body.cg_to_center * s, x[IPSI]);
}

double Circuit::clearance(double t, const State& x) const {
    const Corners ego = ego_corners(x);
    double best = std::numeric_limits<double>::infinity();
    for (const TrafficCar& car : traffic) {
        const TrafficPose p = traffic_pose(car, t);
        const double gap = std::hypot(p.x - x[IX], p.y - x[IY]) - body.length;
        if (gap < best)  // only then can the bodies be closer than the best so far
            best = std::min(best, polygon_distance(ego, body_corners(p.x, p.y, p.psi)));
    }
    return best;
}

double Circuit::road_margin(const State& x) const {
    const Corners corners = ego_corners(x);
    double furthest = 0.0;
    for (int i = 0; i < 4; ++i)
        furthest = std::max(furthest, std::fabs(track.project(corners(i, 0), corners(i, 1)).second));
    return road_half_width() - furthest;
}

// Separating-axis test for two convex polygons.
static bool overlap(const Corners& P, const Corners& Q) {
    for (const Corners* A : {&P, &Q}) {
        for (int i = 0; i < 4; ++i) {
            const Eigen::RowVector2d edge = A->row(i) - A->row((i + 3) % 4);
            const Eigen::Vector2d normal(-edge[1], edge[0]);
            const Eigen::Vector4d p = P * normal, q = Q * normal;
            if (p.maxCoeff() < q.minCoeff() || q.maxCoeff() < p.minCoeff()) return false;
        }
    }
    return true;
}

static double point_segment_distance(const Eigen::RowVector2d& a, const Eigen::RowVector2d& b0,
                                     const Eigen::RowVector2d& b1) {
    const Eigen::RowVector2d d = b1 - b0;
    const double s = std::clamp((a - b0).dot(d) / d.dot(d), 0.0, 1.0);
    return (a - b0 - s * d).norm();
}

double polygon_distance(const Corners& P, const Corners& Q) {
    if (overlap(P, Q)) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& [A, B] : {std::pair{&P, &Q}, std::pair{&Q, &P}})
        for (int a = 0; a < 4; ++a)
            for (int i = 0; i < 4; ++i)
                best = std::min(best, point_segment_distance(A->row(a), B->row(i), B->row((i + 3) % 4)));
    return best;
}

}  // namespace sim
