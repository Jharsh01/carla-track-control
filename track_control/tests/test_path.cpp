// Path: building from corners and from waypoints, projection, offset lines, open ends.
#include <cmath>
#include <stdexcept>

#include "check.hpp"
#include "track_control/path.hpp"

using namespace tc;

static void rounded_rectangle() {
    // 200 x 100 m, driven anticlockwise, corner radius 20 m
    const Path path = Path::from_corners({{0, 0}, {200, 0}, {200, 100}, {0, 100}}, {20, 20, 20, 20}, {50, 0});
    CHECK(path.closed());
    const double expected = 2 * (200 + 100) - 8 * 20 + 2 * M_PI * 20;
    CHECK_NEAR(path.length(), expected, 1e-9);
    CHECK_NEAR(path.max_curvature(), 1.0 / 20, 1e-12);
    CHECK_NEAR(path.curvature(10.0), 0.0, 0.0);
    CHECK_NEAR(path.curvature(130.0 + 10.0), 1.0 / 20, 1e-12);  // first corner starts 130 m from the start

    // round trip: pose then project
    for (double s = 0.0; s < path.length(); s += 7.3) {
        for (double e : {-3.0, 0.0, 2.5}) {
            const Pose q = path.pose(s, e);
            const auto [s2, e2] = path.project(q.x, q.y);
            CHECK_NEAR(path.difference(s2, s), 0.0, 1e-6);
            CHECK_NEAR(e2, e, 1e-6);
        }
    }
    // wrap and signed difference round the lap
    CHECK_NEAR(path.wrap(path.length() + 1.0), 1.0, 1e-9);
    CHECK_NEAR(path.difference(1.0, path.length() - 1.0), 2.0, 1e-9);

    // a line 2 m to the left (inside of an anticlockwise loop) is shorter by 2 * 2 pi * 2 m... per full turn
    CHECK_NEAR(path.offset_length(path.length(), 2.0), path.length() - 2.0 * 2.0 * M_PI, 1e-9);
    CHECK_NEAR(path.offset_to_s(path.offset_length(123.0, 2.0), 2.0), 123.0, 1e-9);
}

static void circle_from_waypoints() {
    const double R = 50.0;
    std::vector<Point> points;
    for (int i = 0; i < 72; ++i) {
        const double a = 2.0 * M_PI * i / 72;
        points.push_back({R * std::cos(a), R * std::sin(a)});
    }
    const Path path = Path::from_waypoints(points, true);
    CHECK(path.closed());
    CHECK_NEAR(path.length(), 2.0 * M_PI * R, 0.01);
    for (double s = 0.0; s < path.length(); s += 3.1) CHECK_NEAR(path.curvature(s), 1.0 / R, 0.001 / R);
    // passes through every waypoint
    for (const Point& q : points) CHECK_NEAR(path.project(q[0], q[1]).second, 0.0, 1e-6);
}

static void open_polyline() {
    // an L: 100 m east, then 100 m north; the biarcs round the corner
    const Path path = Path::from_waypoints({{0, 0}, {50, 0}, {100, 0}, {100, 50}, {100, 100}}, false);
    CHECK(!path.closed());
    const Pose end = path.pose(path.length());
    CHECK_NEAR(end.x, 100.0, 1e-6);
    CHECK_NEAR(end.y, 100.0, 1e-6);
    CHECK_NEAR(end.psi, M_PI / 2, 1e-9);
    // straight on past the end, clamped projection, no wrap
    const Pose beyond = path.pose(path.length() + 10.0);
    CHECK_NEAR(beyond.x, 100.0, 1e-6);
    CHECK_NEAR(beyond.y, 110.0, 1e-6);
    CHECK_NEAR(path.wrap(-5.0), 0.0, 0.0);
    CHECK_NEAR(path.difference(10.0, path.length() - 1.0), 11.0 - path.length(), 1e-9);
    CHECK_NEAR(path.curvature(path.length() + 1.0), 0.0, 0.0);
    for (double s = 0.0; s <= path.length(); s += 2.0) {
        const Pose q = path.pose(s);
        CHECK_NEAR(path.project(q.x, q.y).first, s, 1e-6);
    }
}

static void pieces() {
    // a full circle of radius 10 from pieces closes; a gap does not
    const Path circle = Path::from_pieces({0, 0, 0}, {2 * M_PI * 10}, {0.1}, true);
    CHECK_NEAR(circle.length(), 2 * M_PI * 10, 1e-9);
    bool threw = false;
    try {
        (void)Path::from_pieces({0, 0, 0}, {10.0}, {0.0}, true);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

int main() {
    rounded_rectangle();
    circle_from_waypoints();
    open_polyline();
    pieces();
    return check_report("test_path");
}
