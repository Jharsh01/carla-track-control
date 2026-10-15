#include "track_control/path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tc {

double pymod(double a, double b) {
    const double r = std::fmod(a, b);
    return (r != 0.0 && (r < 0.0) != (b < 0.0)) ? r + b : r;
}

double wrap_angle(double angle) { return pymod(angle + M_PI, 2.0 * M_PI) - M_PI; }

Pose advance(const Pose& from, double kappa, double ds) {
    const double end = from.psi + kappa * ds;
    if (kappa == 0.0) return {from.x + ds * std::cos(from.psi), from.y + ds * std::sin(from.psi), end};
    return {from.x + (std::sin(end) - std::sin(from.psi)) / kappa,
            from.y - (std::cos(end) - std::cos(from.psi)) / kappa, end};
}

// Linear interpolation of the points (xp, fp) at x, clamped at the ends (as numpy.interp).
static double interp(double x, const std::vector<double>& xp, const std::vector<double>& fp) {
    if (x <= xp.front()) return fp.front();
    if (x >= xp.back()) return fp.back();
    const size_t i = std::upper_bound(xp.begin(), xp.end(), x) - xp.begin();
    return fp[i - 1] + (x - xp[i - 1]) / (xp[i] - xp[i - 1]) * (fp[i] - fp[i - 1]);
}

void Path::add(double piece_length, double kappa) {
    if (piece_length > 1e-9) {
        seg_length_.push_back(piece_length);
        seg_kappa_.push_back(kappa);
    }
}

void Path::finish(const Pose& start, bool closed) {
    if (seg_length_.empty()) throw std::invalid_argument("a path needs at least one piece of nonzero length");
    closed_ = closed;
    seg_s_.assign(1, 0.0);
    for (double piece_length : seg_length_) seg_s_.push_back(seg_s_.back() + piece_length);
    length_ = seg_s_.back();
    Pose at = start;
    seg_pose_.clear();
    for (size_t k = 0; k < seg_length_.size(); ++k) {
        seg_pose_.push_back(at);
        at = advance(at, seg_kappa_[k], seg_length_[k]);
    }
    if (closed && std::hypot(at.x - start.x, at.y - start.y) > 1e-6 * std::max(1.0, length_))
        throw std::invalid_argument("path does not close");
}

Path Path::from_corners(const std::vector<Point>& corners, const std::vector<double>& radii, const Point& start) {
    const int n = static_cast<int>(corners.size());
    if (n < 3 || radii.size() != corners.size()) throw std::invalid_argument("need 3 or more corners, one radius each");
    std::vector<double> edge_len(n), heading(n), turn(n), tangent(n), straight(n);
    for (int i = 0; i < n; ++i) {  // edge i runs from corner i to corner i + 1
        const double dx = corners[(i + 1) % n][0] - corners[i][0], dy = corners[(i + 1) % n][1] - corners[i][1];
        edge_len[i] = std::hypot(dx, dy);
        heading[i] = std::atan2(dy, dx);
    }
    for (int i = 0; i < n; ++i) {
        turn[i] = wrap_angle(heading[i] - heading[(i + n - 1) % n]);  // heading change at corner i
        tangent[i] = radii[i] * std::tan(0.5 * std::fabs(turn[i]));  // corner to arc end
    }
    for (int i = 0; i < n; ++i) {
        straight[i] = edge_len[i] - tangent[i] - tangent[(i + 1) % n];
        if (straight[i] < -1e-9) throw std::invalid_argument("corner radii do not fit on the edges");
    }
    const double to_start = std::hypot(start[0] - corners[0][0], start[1] - corners[0][1]) - tangent[0];
    if (!(0.0 <= to_start && to_start <= straight[0]))
        throw std::invalid_argument("start must lie on the straight part of the first edge");

    // pieces from the start line round to the start line
    Path path;
    path.add(straight[0] - to_start, 0.0);
    for (int k = 1; k <= n; ++k) {
        const int i = k % n;
        path.add(radii[i] * std::fabs(turn[i]), (turn[i] > 0.0 ? 1.0 : turn[i] < 0.0 ? -1.0 : 0.0) / radii[i]);
        path.add(i ? straight[i] : to_start, 0.0);
    }
    path.finish({start[0], start[1], heading[0]}, true);
    return path;
}

Path Path::from_pieces(const Pose& start, const std::vector<double>& lengths, const std::vector<double>& curvatures,
                       bool closed) {
    if (lengths.size() != curvatures.size()) throw std::invalid_argument("one curvature per piece");
    Path path;
    for (size_t k = 0; k < lengths.size(); ++k) path.add(lengths[k], curvatures[k]);
    path.finish(start, closed);
    return path;
}

namespace {

struct Vec {
    double x, y;
};
Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y}; }
Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y}; }
Vec operator*(double k, Vec a) { return {k * a.x, k * a.y}; }
double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
double cross(Vec a, Vec b) { return a.x * b.y - a.y * b.x; }
double norm(Vec a) { return std::hypot(a.x, a.y); }
Vec unit(Vec a) { return (1.0 / norm(a)) * a; }

// Length and curvature of the arc that leaves `from` along the unit tangent `t` and reaches `to`.
std::pair<double, double> arc_to(Vec from, Vec t, Vec to) {
    const Vec chord = to - from;
    const double c = norm(chord);
    if (c < 1e-12) return {0.0, 0.0};
    const double phi = std::atan2(cross(t, chord), dot(t, chord));  // tangent to chord; the arc turns by 2 phi
    if (std::fabs(phi) < 1e-9) return {c, 0.0};
    return {c * phi / std::sin(phi), 2.0 * std::sin(phi) / c};
}

}  // namespace

Path Path::from_waypoints(const std::vector<Point>& points, bool closed) {
    std::vector<Vec> p;
    for (const Point& q : points)  // repeated points carry no direction
        if (p.empty() || norm(Vec{q[0], q[1]} - p.back()) > 1e-6) p.push_back({q[0], q[1]});
    if (closed && p.size() > 1 && norm(p.front() - p.back()) < 1e-6) p.pop_back();
    const int n = static_cast<int>(p.size());
    if (n < (closed ? 3 : 2)) throw std::invalid_argument("too few distinct waypoints for a path");

    // tangent at each point: along the chord between its neighbours (the segment, at an open end)
    std::vector<Vec> t(n);
    for (int i = 0; i < n; ++i) {
        if (!closed && i == 0) t[i] = unit(p[1] - p[0]);
        else if (!closed && i == n - 1) t[i] = unit(p[n - 1] - p[n - 2]);
        else t[i] = unit(p[(i + 1) % n] - p[(i + n - 1) % n]);
    }

    // a biarc between each pair of points: two arcs meeting with a common tangent, equal distances to the join
    Path path;
    const int edges = closed ? n : n - 1;
    for (int i = 0; i < edges; ++i) {
        const Vec p0 = p[i], p1 = p[(i + 1) % n], t0 = t[i], t1 = t[(i + 1) % n];
        const Vec v = p1 - p0, tt = t0 + t1;
        const double denominator = 2.0 * (1.0 - dot(t0, t1));
        double d;
        if (denominator < 1e-12) {  // parallel tangents
            const double v_t1 = dot(v, t1);
            d = std::fabs(v_t1) > 1e-12 ? dot(v, v) / (4.0 * v_t1) : 0.5 * norm(v);
        } else {
            const double v_t = dot(v, tt);
            d = (-v_t + std::sqrt(v_t * v_t + denominator * dot(v, v))) / denominator;
        }
        const Vec join = 0.5 * (p0 + p1 + d * (t0 - t1));
        const auto [len0, k0] = arc_to(p0, t0, join);
        const Pose mid = advance({p0.x, p0.y, std::atan2(t0.y, t0.x)}, k0, len0);
        const auto [len1, k1] = arc_to(join, {std::cos(mid.psi), std::sin(mid.psi)}, p1);
        path.add(len0, k0);
        path.add(len1, k1);
    }
    path.finish({p[0].x, p[0].y, std::atan2(t[0].y, t[0].x)}, closed);
    return path;
}

std::pair<int, double> Path::piece(double s) const {
    s = wrap(s);
    const int last = static_cast<int>(seg_length_.size()) - 1;
    const int k = std::clamp(
        static_cast<int>(std::upper_bound(seg_s_.begin(), seg_s_.end(), s) - seg_s_.begin()) - 1, 0, last);
    return {k, s - seg_s_[k]};
}

double Path::wrap(double s) const { return closed_ ? pymod(s, length_) : std::clamp(s, 0.0, length_); }

double Path::difference(double to, double from) const {
    return closed_ ? pymod(to - from + 0.5 * length_, length_) - 0.5 * length_ : to - from;
}

double Path::curvature(double s) const {
    if (!closed_ && (s < 0.0 || s > length_)) return 0.0;
    return seg_kappa_[piece(s).first];
}

double Path::max_curvature() const {
    double out = 0.0;
    for (double kappa : seg_kappa_) out = std::max(out, std::fabs(kappa));
    return out;
}

Pose Path::pose(double s, double e_y) const {
    Pose at;
    if (!closed_ && s > length_) {  // straight on past the end
        const Pose end = advance(seg_pose_.back(), seg_kappa_.back(), seg_length_.back());
        at = advance(end, 0.0, s - length_);
    } else if (!closed_ && s < 0.0) {
        at = advance(seg_pose_.front(), 0.0, s);
    } else {
        const auto [k, ds] = piece(s);
        at = advance(seg_pose_[k], seg_kappa_[k], ds);
    }
    return {at.x - e_y * std::sin(at.psi), at.y + e_y * std::cos(at.psi), at.psi};
}

std::pair<double, double> Path::project(double x, double y) const {
    double best = std::numeric_limits<double>::infinity(), best_s = 0.0, best_e = 0.0;
    for (size_t k = 0; k < seg_length_.size(); ++k) {
        const Pose& p0 = seg_pose_[k];
        const double kappa = seg_kappa_[k], piece_length = seg_length_[k];
        double ds;
        if (kappa == 0.0) {
            ds = (x - p0.x) * std::cos(p0.psi) + (y - p0.y) * std::sin(p0.psi);
        } else {
            // angle swept round the arc's centre, measured in the direction of travel
            const double cx = p0.x - std::sin(p0.psi) / kappa, cy = p0.y + std::cos(p0.psi) / kappa;
            const double swept = (kappa > 0.0 ? 1.0 : -1.0) *
                                 wrap_angle(std::atan2(y - cy, x - cx) - std::atan2(p0.y - cy, p0.x - cx));
            ds = swept / std::fabs(kappa);
            // nearer the far end, the other way round
            if (ds < -0.5 * (2.0 * M_PI / std::fabs(kappa) - piece_length)) ds += 2.0 * M_PI / std::fabs(kappa);
        }
        ds = std::clamp(ds, 0.0, piece_length);
        const Pose q = advance(p0, kappa, ds);
        const double e_y = -(x - q.x) * std::sin(q.psi) + (y - q.y) * std::cos(q.psi);
        const double dist = std::hypot(x - q.x, y - q.y);
        if (dist < best) {
            best = dist;
            best_s = seg_s_[k] + ds;
            best_e = e_y;
        }
    }
    return {wrap(best_s), best_e};
}

std::vector<double> Path::cumulative(double e_y) const {
    std::vector<double> out(1, 0.0);
    for (size_t k = 0; k < seg_length_.size(); ++k)  // an inside line is shorter
        out.push_back(out.back() + seg_length_[k] * (1.0 - seg_kappa_[k] * e_y));
    return out;
}

double Path::offset_length(double s, double e_y) const { return interp(s, seg_s_, cumulative(e_y)); }

double Path::offset_to_s(double distance, double e_y) const {
    const std::vector<double> c = cumulative(e_y);
    return interp(closed_ ? pymod(distance, c.back()) : distance, c, seg_s_);
}

}  // namespace tc
