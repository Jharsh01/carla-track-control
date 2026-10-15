// Reference path the controllers follow: a chain of straights and circular arcs.
//
// The path is parametrised by arc length s [m] from its start. Path coordinates
// are (s, e_y), e_y being the lateral offset from the path, positive to the
// left of the direction of travel. Curvature is positive in left turns.
//
// A path is closed (a lap: s wraps round at `length()`) or open (s is clamped
// to [0, length()], and the path continues straight past its end, so a
// lookahead point beyond the end is still defined).
//
// Three ways to build one:
// - `from_corners`: a closed polygon whose corners are rounded with a radius
//   each (how the race track is drawn);
// - `from_waypoints`: points along a route, e.g. CARLA waypoints, joined by
//   biarcs, so the path passes through every point with continuous heading;
// - `from_pieces`: the pieces themselves, from a start pose.
#pragma once

#include <array>
#include <utility>
#include <vector>

namespace tc {

struct Pose {
    double x, y, psi;
};

using Point = std::array<double, 2>;

class Path {
public:
    // `corners` are the polygon vertices in driving order, `radii` the corner
    // radius at each, and `start` a point on the edge from corner 0 to corner 1
    // where s = 0.
    static Path from_corners(const std::vector<Point>& corners, const std::vector<double>& radii,
                             const Point& start);
    // Through `points` in order. A closed path also joins the last point to the first.
    static Path from_waypoints(const std::vector<Point>& points, bool closed);
    // Constant-curvature pieces of the given lengths, from `start`. A closed path must end where it starts.
    static Path from_pieces(const Pose& start, const std::vector<double>& lengths,
                            const std::vector<double>& curvatures, bool closed);

    double length() const { return length_; }
    bool closed() const { return closed_; }

    double curvature(double s) const;
    double max_curvature() const;  // largest |curvature| along the path [1/m]
    // World x, y and path heading at arc length `s`, offset `e_y` to the left.
    Pose pose(double s, double e_y = 0.0) const;
    // Path coordinates (s, e_y) of the world point (x, y): the nearest path point.
    std::pair<double, double> project(double x, double y) const;
    // Distance travelled from the start to `s` along the line offset by `e_y` (an inside line is shorter).
    double offset_length(double s, double e_y) const;
    // Inverse of `offset_length`; wraps round the lap on a closed path.
    double offset_to_s(double distance, double e_y) const;

    // `s` in [0, length): wrapped round the lap on a closed path, clamped on an open one.
    double wrap(double s) const;
    // Signed distance along the path from `from` to `to`: the shorter way round a closed path.
    double difference(double to, double from) const;

    // The pieces: length, curvature, arc length and pose at the start of each.
    const std::vector<double>& piece_lengths() const { return seg_length_; }
    const std::vector<double>& piece_curvatures() const { return seg_kappa_; }
    const std::vector<double>& piece_starts() const { return seg_s_; }  // one more entry: the path length
    const std::vector<Pose>& piece_poses() const { return seg_pose_; }

private:
    Path() = default;
    void add(double piece_length, double kappa);  // append a piece (skips empty ones)
    void finish(const Pose& start, bool closed);  // arc lengths and poses of the pieces
    std::pair<int, double> piece(double s) const;  // index of the piece at s and the distance into it
    std::vector<double> cumulative(double e_y) const;

    double length_ = 0.0;
    bool closed_ = true;
    std::vector<double> seg_length_, seg_kappa_, seg_s_;
    std::vector<Pose> seg_pose_;
};

// Remainder with the sign of the divisor, as Python's % operator.
double pymod(double a, double b);
// Angle wrapped to [-pi, pi).
double wrap_angle(double angle);
// Pose after travelling `ds` from `from` at constant curvature `kappa`.
Pose advance(const Pose& from, double kappa, double ds);

}  // namespace tc
