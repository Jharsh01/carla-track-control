#include "track_control/nmpc.hpp"

#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <chrono>
#include <ctime>
#include <functional>
#include <map>
#include <mutex>
#include <sstream>
#include <cmath>
#include <stdexcept>

#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"

namespace tc {

using casadi::Slice;
using casadi::SX;
static constexpr double FAR = 1e4;  // s offset that puts an unused obstacle slot out of reach [m]

namespace {

StateOf<SX> state_of(const SX& x) { return {x(IX), x(IY), x(IPSI), x(IUX), x(IUY), x(IR), x(IDELTA)}; }

// The steps of the horizon: o.h_fine for the first o.fine seconds, o.h after them.
std::vector<double> make_steps(const NMPC::Options& o) {
    if (!(o.h > 0.0) || !(o.horizon > 0.0) || o.h_fine < 0.0 || o.fine < 0.0)
        throw std::invalid_argument("NMPC: the steps and the horizon must be positive");
    const int n_fine = o.h_fine > 0.0 ? static_cast<int>(std::lround(std::min(o.fine, o.horizon) / o.h_fine)) : 0;
    const int n = static_cast<int>(std::lround((o.horizon - n_fine * o.h_fine) / o.h));
    std::vector<double> steps(n_fine, o.h_fine);
    steps.insert(steps.end(), std::max(0, n), o.h);
    if (steps.empty()) throw std::invalid_argument("NMPC: a horizon shorter than one step");
    return steps;
}

std::vector<double> start_times(const std::vector<double>& steps) {
    std::vector<double> times{0.0};
    for (double dt : steps) times.push_back(times.back() + dt);
    return times;
}

double tightest_corner_speed(const Path& path, const Road& road, const VehicleParams& p) {
    const double kappa_max = path.max_curvature();
    return kappa_max > 0.0 ? std::sqrt(road.mu * p.g / kappa_max) : INF;
}

// The parameters planned with: cornering stiffness constant at the static load. Stiffness following
// the load couples Fx into the tire model and about doubles the cost of a solve.
VehicleParams constant_stiffness(VehicleParams p) {
    const double Fzf = p.m * p.g * p.b / p.L(), Fzr = p.m * p.g * p.a / p.L();
    p.C_alpha_f = p.C_alpha_f_at(Fzf);
    p.C_alpha_r = p.C_alpha_r_at(Fzr);
    p.C_alpha_f_load = p.C_alpha_r_load = LoadTable();
    return p;
}

// One step of the horizon as the cost and constraint terms see it: the state it ends in, its input and
// slacks, and the parameters it is planned with.
struct Stage {
    SX x, u, slack;  // x: the state at the end of the step
    SX kappa, v_ref, v_lim, e_ref;  // curvature, speed asked for, speed limit [m/s], lateral offset asked for [m]
    std::vector<std::array<SX, 5>> obstacles;  // s, e_y, the ellipse's half axes [m] and 1 (0: unused) per car and zone
    SX e_max, slip;  // furthest the body's ends may be from the path [m], tan of the sideslip limit
    Body body;
};

// The cost: a sum of named terms, each weighted by a parameter, Options giving the default and a Directive
// perhaps another. A new term is one more entry here, and with it a weight a directive can set.
struct CostTerm {
    const char* name;
    double NMPC::Options::*weight;
    std::function<SX(const Stage&)> f;
};
const std::vector<CostTerm>& cost_terms() {
    static const std::vector<CostTerm> terms = {
        {"speed", &NMPC::Options::w_speed, [](const Stage& g) { return sq(g.x(IUX) - g.v_ref); }},
        {"lane", &NMPC::Options::w_lane, [](const Stage& g) { return sq(g.x(IY) - g.e_ref); }},
        {"lat", &NMPC::Options::w_lat,
         [](const Stage& g) {
             const SX v_lat = g.x(IUX) * sin(g.x(IPSI)) + g.x(IUY) * cos(g.x(IPSI));
             return sq(v_lat) + sq(g.x(IR) - g.kappa * g.x(IUX));
         }},
        {"accel", &NMPC::Options::w_accel, [](const Stage& g) { return sq(g.u(IFX)); }},
        {"rate", &NMPC::Options::w_rate, [](const Stage& g) { return sq(g.u(IRATE)); }},
        {"slack", &NMPC::Options::w_slack, [](const Stage& g) { return sum1(g.slack) + sumsqr(g.slack); }},
    };
    return terms;
}

// The constraints in groups, each group softened by one slack per step: every expression <= that slack. A new
// group is one more entry here and one more slack (NMPC::NS).
struct ConstraintGroup {
    const char* name;
    std::function<std::vector<SX>(const Stage&)> f;
};
const std::vector<ConstraintGroup>& constraint_groups() {
    static const std::vector<ConstraintGroup> groups = {
        {"obstacle",
         [](const Stage& g) {  // outside an ellipse around each car and zone, in ego centre-of-gravity coordinates
             std::vector<SX> out;
             for (const auto& [s, e, a, b, on] : g.obstacles)  // an unused one a constant -1
                 out.push_back(on * (1.0 - sq((g.x(IX) - g.body.cg_to_center - s) / a) - sq((g.x(IY) - e) / b)) +
                               (on - 1.0));
             return out;
         }},
        {"edge",
         [](const Stage& g) {  // road edges, at the front and rear ends of the body
             std::vector<SX> out;
             const Body& b = g.body;
             for (double arm : {0.5 * b.length - b.cg_to_center, -(0.5 * b.length + b.cg_to_center)}) {
                 const SX e_end = g.x(IY) + arm * sin(g.x(IPSI));
                 out.push_back(e_end - g.e_max);
                 out.push_back(-g.e_max - e_end);
             }
             return out;
         }},
        {"sideslip",
         [](const Stage& g) {
             return std::vector<SX>{g.x(IUY) - g.slip * g.x(IUX), -g.x(IUY) - g.slip * g.x(IUX)};
         }},
        {"speed", [](const Stage& g) { return std::vector<SX>{g.x(IUX) - g.v_lim}; }},
    };
    return groups;
}

}  // namespace

struct NMPC::Impl {
    casadi::Function cold, warm_solver;
    std::vector<double> lbw, ubw, lbg, ubg;
    int n_in = 0;  // inequality constraints per stage
    // previous primal and dual solution, planned at time t, to start the next solve from
    std::vector<double> x0, lam_x0, lam_g0;
    double t = 0.0;
    bool warm = false;
    std::map<std::string, Range> weight_range, limit_range;
    // the directive in force, set from any thread
    std::mutex mutex;
    Directive directive;
    bool directed = false;
};

NMPC::NMPC(Path path_, Road road_, VehicleParams params, Body body_, Options o)
    : path(std::move(path_)),
      road(road_),
      p(constant_stiffness(params)),
      body(body_),
      opt(o),
      h(o.h),
      steps(make_steps(o)),
      N(static_cast<int>(steps.size())),
      times(start_times(steps)),
      unseen_speed(std::min(o.unseen_speed.value_or(tightest_corner_speed(path, road_, params)), road_.v_max)),
      impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    if (static_cast<int>(constraint_groups().size()) != NS)
        throw std::logic_error("NMPC: NS must be the number of constraint groups");
    const int n_cars = o.n_cars, n_zones = o.n_zones;

    // What a directive may set, and within what: weights from a tenth to ten times their default (the slack
    // weight only up, so constraints never get softer), and limits that only ever make the car more careful.
    for (const CostTerm& term : cost_terms()) {
        const double d = o.*term.weight;
        m.weight_range[term.name] = {std::string(term.name) == "slack" ? d : 0.1 * d, 10.0 * d, d};
    }
    const double lane_max = std::max(road.half_width - 0.5 * road.lane_width, std::fabs(o.lane_offset));
    m.limit_range = {
        {"speed_cap", {3.0, road.v_max, road.v_max}},
        {"lane_offset", {-lane_max, lane_max, o.lane_offset}},
        // up to where the cars' ellipse, as wide as a lane, still holds the body and the margin
        {"margin_lat", {o.margin_lat, std::max(o.margin_lat, road.lane_width - 0.45 - body.width), o.margin_lat}},
        {"margin_long", {o.margin_long, o.margin_long + 30.0, o.margin_long}},
        // up to where the body still fits at the centre of the outer lanes
        {"edge_margin",
         {o.edge_margin, std::max(o.edge_margin, road.half_width - 0.5 * body.width - lane_max), o.edge_margin}},
        {"sideslip_max", {std::min(2.0 * M_PI / 180.0, o.sideslip_max), o.sideslip_max, o.sideslip_max}},
    };

    // Parameters, set at every solve: the measured state; per step the curvature, speed limit, speed and lateral
    // offset asked for; the cost weights; the cars' and zones' positions at each step, lateral offsets and
    // ellipse half axes, which zones are in use; the road-edge and sideslip limits.
    const int n_terms = static_cast<int>(cost_terms().size());
    const SX x_init = SX::sym("x_init", NX);
    const SX kappa = SX::sym("kappa", N);
    const SX v_lim = SX::sym("v_lim", N);
    const SX v_ref = SX::sym("v_ref", N);
    const SX e_ref = SX::sym("e_ref", N);
    const SX weight = SX::sym("weight", n_terms);
    const SX car_s = SX::sym("car_s", n_cars, N);
    const SX car_e = SX::sym("car_e", n_cars);
    const SX car_ab = SX::sym("car_ab", 2);
    const SX zone_s = SX::sym("zone_s", n_zones, N);
    const SX zone_e = SX::sym("zone_e", n_zones);
    const SX zone_ab = SX::sym("zone_ab", n_zones, 2);
    const SX zone_on = SX::sym("zone_on", n_zones);
    const SX e_max = SX::sym("e_max");
    const SX slip = SX::sym("slip");
    const SX par = SX::vertcat({x_init, kappa, v_lim, v_ref, e_ref, weight, vec(car_s), car_e, car_ab, vec(zone_s),
                                zone_e, vec(zone_ab), zone_on, e_max, slip});

    // decision variables stage by stage, [x_0 u_0 s_0  x_1 u_1 s_1 ... x_N], each constraint and cost term
    // depending on one stage's variables only: what a structure-exploiting solver (FATROP) needs, and the
    // same problem for IPOPT. The longitudinal input is the acceleration Fx / m, for scaling.
    std::vector<SX> w, g;
    std::vector<bool> equality;
    SX J = 0.0;
    std::vector<SX> xs;
    for (int k = 0; k <= N; ++k) xs.push_back(SX::sym("x_" + std::to_string(k), NX));

    const VehicleParams& pp = p;
    for (int k = 0; k < N; ++k) {
        const SX xk = xs[k], uk = SX::sym("u_" + std::to_string(k), NU), sk = SX::sym("s_" + std::to_string(k), NS);
        w.insert(w.end(), {xk, uk, sk});
        const SX kap = kappa(k);
        auto f = [&pp, &kap](const SX& z, const SX& v) {
            // with psi = e_psi
            const StateOf<SX> d = dynamics<SX>(state_of(z), {v(IRATE), pp.m * v(IFX)}, pp);
            const SX s_dot = d[IX] / (1.0 - kap * z(IY));
            return SX::vertcat({s_dot, d[IY], d[IPSI] - kap * s_dot, d[IUX], d[IUY], d[IR], d[IDELTA]});
        };
        // the next state as this stage predicts it: equal to x_{k+1} once the gap is closed
        const SX nxt = rk4_step(f, xk, uk, steps[k]);
        g.push_back(xs[k + 1] - nxt);  // gap closing first
        equality.insert(equality.end(), NX, true);
        if (k == 0) {
            g.push_back(xk - x_init);
            equality.insert(equality.end(), NX, true);
        }

        Stage stage{nxt, uk, sk, kap, v_ref(k), v_lim(k), e_ref(k), {}, e_max, slip, body};
        for (int j = 0; j < n_cars; ++j) stage.obstacles.push_back({car_s(j, k), car_e(j), car_ab(0), car_ab(1), 1.0});
        for (int j = 0; j < n_zones; ++j)
            stage.obstacles.push_back({zone_s(j, k), zone_e(j), zone_ab(j, 0), zone_ab(j, 1), zone_on(j)});
        // each term weighted by the step's length, the cost a sum over time whatever the steps
        for (int i = 0; i < n_terms; ++i) J += (steps[k] / h) * weight(i) * cost_terms()[i].f(stage);
        std::vector<SX> in;  // each entry <= 0
        for (int i = 0; i < NS; ++i)
            for (const SX& c : constraint_groups()[i].f(stage)) in.push_back(c - sk(i));
        g.push_back(SX::vertcat(in));
        equality.insert(equality.end(), in.size(), false);
    }
    w.push_back(xs[N]);

    for (bool e : equality) {
        m.lbg.push_back(e ? 0.0 : -INF);
        m.ubg.push_back(0.0);
    }
    for (int k = 0; k <= N; ++k) {
        const bool bounded = k > 0;
        for (int i = 0; i < NX; ++i) {
            // the tire model is not meant for speeds below 3 m/s
            m.lbw.push_back(bounded && i == IDELTA ? -p.delta_max : bounded && i == IUX ? 3.0 : -INF);
            m.ubw.push_back(bounded && i == IDELTA ? p.delta_max : INF);
        }
        if (k == N) break;
        m.lbw.insert(m.lbw.end(), {-p.delta_rate_max, 0.0});  // force limits are set per solve
        m.ubw.insert(m.ubw.end(), {p.delta_rate_max, 0.0});
        m.lbw.insert(m.lbw.end(), NS, 0.0);
        m.ubw.insert(m.ubw.end(), NS, INF);
    }

    m.n_in = static_cast<int>(g.back().numel());

    // Two instances of the solver: a cold one for the first solve and a warm one that, with `warm_duals`,
    // takes the multipliers of the previous solve as its starting point (warm_start_init_point), which with
    // no previous multipliers would start from zero duals.
    const casadi::SXDict nlp = {{"x", SX::vertcat(w)}, {"p", par}, {"f", J}, {"g", SX::vertcat(g)}};
    auto make = [&](bool warm) {
        warm = warm && o.warm_duals;
        casadi::Dict opts = {{"print_time", false}};
        if (o.solver == "fatrop") {
            // the stages are found from the variable and constraint order above; `equality` tells
            // FATROP which rows of g are equalities, as it does not read that from lbg and ubg
            opts["structure_detection"] = "auto";
            opts["equality"] = equality;
            opts["fatrop"] = casadi::Dict{{"print_level", 0}, {"max_iter", o.max_iter}, {"tol", o.tol},
                                          {"warm_start_init_point", warm}};
        } else if (o.solver == "ipopt") {
            opts["ipopt.print_level"] = 0;
            opts["ipopt.sb"] = "yes";
            opts["ipopt.max_iter"] = o.max_iter;
            opts["ipopt.tol"] = o.tol;
            opts["ipopt.warm_start_init_point"] = warm ? "yes" : "no";
            if (o.max_wall_time > 0.0) opts["ipopt.max_wall_time"] = o.max_wall_time;
        } else {
            throw std::invalid_argument("NMPC: unknown solver '" + o.solver + "' (fatrop or ipopt)");
        }
        return casadi::nlpsol(warm ? "track_nmpc_warm" : "track_nmpc", o.solver, nlp, opts);
    };
    m.cold = make(false);
    m.warm_solver = make(true);
}

NMPC::~NMPC() = default;

State NMPC::model_state(const State& x) const {
    State z = x;
    const PathState ps = path_state(path, x);
    z[IX] = ps.s;
    z[IY] = ps.e_y;
    z[IPSI] = ps.e_psi;
    return z;
}

Input NMPC::compute(const Observation& obs) {
    Impl& m = *impl_;
    const int n_cars = opt.n_cars;
    const State& x = obs.x;
    const State z0 = model_state(x);
    const double s0 = z0[IX];
    // the road as known from s0: `s` counts on from s0 without wrapping
    auto known_curvature = [&](double s) { return path.curvature(std::min(s, s0 + opt.road_range)); };

    // The starting point: at first the measured state carried on at constant speed, after that the previous
    // solution moved on by the time since it was planned: stage k takes the previous solution at times[k] plus
    // that time (interpolated between its stages). The stages beyond its horizon repeat its last input and
    // multipliers, with the last state coasting on at its speed. Each stage's inequality multipliers stay
    // nonnegative, so the dual start is valid.
    double x0_gap = 0.0;  // of the shifted guess, before x_0 is set to the measured state
    if (m.warm) {
        const double since = std::max(0.0, obs.t - m.t);
        auto index_at = [&](double t) {  // the previous solution's stage at time t, with the fraction past it
            const int j = step_at(t);
            if (j >= N) return N + std::max(0.0, t - times[N]) / steps.back();
            return j + std::max(0.0, t - times[j]) / steps[j];
        };
        auto shifted = [&](int k, auto value_at) {
            const double src = index_at(times[k] + since);
            const int j = static_cast<int>(std::floor(src));
            const double a = src - j;
            return a > 0.0 ? (1.0 - a) * value_at(j) + a * value_at(j + 1) : value_at(j);
        };
        // w and lam_x: stage k's [x u s] at x_at(k), the last stage x only
        auto shift_w = [&](const std::vector<double>& old, bool coast) {
            std::vector<double> out(old.size());
            for (int k = 0; k <= N; ++k)
                for (int i = 0; i < (k < N ? NX + NU + NS : NX); ++i)
                    out[x_at(k) + i] = shifted(k, [&](int j) {
                        const int last = i < NX ? N : N - 1;
                        const double v = old[x_at(std::min(j, last)) + i];
                        return coast && i == IX && j > N ? v + old[x_at(N) + IUX] * steps.back() * (j - N) : v;
                    });
            return out;
        };
        // lam_g: stage k's [gap closing, inequalities], with stage 0's x_0 = x_init rows between them (kept as they were)
        const int n_in = m.n_in;
        auto g_row = [&](int k, int i) { return k == 0 ? (i < NX ? i : NX + i) : 2 * NX + n_in + (k - 1) * (NX + n_in) + i; };
        std::vector<double> lam_g = m.lam_g0;
        for (int k = 0; k < N; ++k)
            for (int i = 0; i < NX + n_in; ++i)
                lam_g[g_row(k, i)] = shifted(k, [&](int j) { return m.lam_g0[g_row(std::min(j, N - 1), i)]; });
        m.x0 = shift_w(m.x0, true);
        m.lam_x0 = shift_w(m.lam_x0, false);
        m.lam_g0 = std::move(lam_g);
        // the s values belong to the previous lap position; move them to this one
        const double shift = s0 - m.x0[x_at(0) + IX];
        for (int k = 0; k <= N; ++k) m.x0[x_at(k) + IX] += shift;
        for (int i = 0; i < NX; ++i) {
            const double d = m.x0[x_at(0) + i] - z0[i];
            x0_gap += d * d;
        }
        std::copy(z0.data(), z0.data() + NX, m.x0.begin() + x_at(0));
    }

    std::vector<double> s_plan(N + 1);
    for (int k = 0; k <= N; ++k)  // constant speed at first, then the shifted previous plan's progress
        s_plan[k] = m.warm ? m.x0[x_at(k) + IX] : s0 + z0[IUX] * times[k];

    // predicted s of the nearest obstacles (n_cars x N, placed around s0) and their lateral offsets
    struct Slot {
        double distance;
        std::vector<double> s;
        double e_y;
    };
    std::vector<Slot> slots;
    const double length = path.length();
    for (const Obstacle& car : obs.obstacles) {
        // constant speed along the line at the car's lateral offset
        const double start = path.offset_length(path.wrap(car.s), car.e_y);
        std::vector<double> s(N);
        for (int k = 0; k < N; ++k) s[k] = path.offset_to_s(start + car.speed * times[k + 1], car.e_y);
        const double ahead = path.difference(s[0], s0);  // signed distance to the car, along the path
        // unwrap the lap wrap out of s - s[0], then place it around s0
        std::vector<double> unwrapped(N);
        double previous = 0.0, offset = 0.0;
        for (int k = 0; k < N; ++k) {
            const double value = s[k] - s[0];
            const double jump = value - previous;
            if (path.closed() && jump > 0.5 * length) offset -= length;
            if (path.closed() && jump < -0.5 * length) offset += length;
            previous = value;
            unwrapped[k] = s0 + ahead + value + offset;
        }
        slots.push_back({std::fabs(ahead), std::move(unwrapped), car.e_y});
    }
    std::stable_sort(slots.begin(), slots.end(), [](const Slot& a, const Slot& b) { return a.distance < b.distance; });
    slots.resize(std::min<size_t>(slots.size(), n_cars));
    const int n_obstacles = static_cast<int>(slots.size());
    while (static_cast<int>(slots.size()) < n_cars) slots.push_back({0.0, std::vector<double>(N, s0 + FAR), 0.0});

    // the settings: the defaults, with what the directive in force changes
    Directive d;
    bool directed;
    {
        std::lock_guard<std::mutex> lock(m.mutex);
        if (m.directed && std::isnan(m.directive.t)) m.directive.t = obs.t;
        if (m.directed && obs.t > m.directive.t + m.directive.ttl) m.directed = false;  // lapsed
        directed = m.directed;
        if (directed) d = m.directive;
    }
    auto setting = [](const std::map<std::string, double>& values, const std::map<std::string, Range>& ranges,
                      const std::string& name) {
        const auto it = values.find(name);
        return it != values.end() ? it->second : ranges.at(name).def;
    };
    auto limit = [&](const char* name) { return setting(d.limits, m.limit_range, name); };
    const double speed_cap = limit("speed_cap"), margin_lat = limit("margin_lat"), margin_long = limit("margin_long");

    // zones: each one's s at each step, placed around s0 like the cars, the nearest n_zones of them
    std::vector<Slot> zones;
    std::vector<std::array<double, 2>> zone_ab;  // ellipse half axes through the corners of the zone grown by the body
    for (const Zone& zone : d.zones) {
        const double ahead = path.difference(path.wrap(zone.s + zone.speed * (obs.t - d.t)), s0);
        std::vector<double> s(N);
        for (int k = 0; k < N; ++k) s[k] = s0 + ahead + zone.speed * times[k + 1];
        zones.push_back({std::fabs(ahead), std::move(s), zone.e_y});
    }
    std::vector<size_t> order(zones.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return zones[a].distance < zones[b].distance; });
    order.resize(std::min<size_t>(order.size(), opt.n_zones));
    std::vector<Slot> zone_slots;
    for (size_t i : order) {
        zone_slots.push_back(zones[i]);
        const Zone& zone = d.zones[i];
        zone_ab.push_back({M_SQRT2 * (0.5 * (zone.length + body.length) + margin_long),
                           M_SQRT2 * (0.5 * (zone.width + body.width) + margin_lat)});
    }
    while (static_cast<int>(zone_slots.size()) < opt.n_zones) {
        // switched off: its constraint a constant -1, which costs the solver nothing (parked out of reach
        // instead, a constraint near -1e8 scales the interior point's slacks badly: half as many iterations again)
        zone_slots.push_back({0.0, std::vector<double>(N, s0 + FAR), 0.0});
        zone_ab.push_back({1.0, 1.0});
    }

    // in the order of the parameters in the constructor
    std::vector<double> par(z0.data(), z0.data() + NX);
    for (int k = 0; k < N; ++k) {  // mean known curvature over each step
        double sum = 0.0;
        for (int i = 0; i < 5; ++i) {
            const double w = i / 4.0;
            sum += known_curvature(s_plan[k] * (1.0 - w) + s_plan[k + 1] * w);
        }
        par.push_back(sum / 5.0);
    }
    for (int k = 0; k < N; ++k)
        par.push_back(std::min(speed_cap, s_plan[k + 1] <= s0 + opt.road_range ? road.v_max : unseen_speed));
    par.insert(par.end(), N, std::min(speed_cap, road.v_max));
    par.insert(par.end(), N, limit("lane_offset"));
    for (const CostTerm& term : cost_terms()) par.push_back(setting(d.weights, m.weight_range, term.name));
    for (int k = 0; k < N; ++k)
        for (int j = 0; j < n_cars; ++j) par.push_back(slots[j].s[k]);
    for (const Slot& slot : slots) par.push_back(slot.e_y);
    {  // the cars' ellipse: as wide as a lane, through the corner of the two bodies side by side with the margins
        const double ly = body.width + margin_lat, lx = body.length + margin_long, B = road.lane_width - 0.4;
        par.insert(par.end(), {lx / std::sqrt(1.0 - (ly / B) * (ly / B)), B});
    }
    for (int k = 0; k < N; ++k)
        for (const Slot& zone : zone_slots) par.push_back(zone.s[k]);
    for (const Slot& zone : zone_slots) par.push_back(zone.e_y);
    for (int i = 0; i < 2; ++i)
        for (const auto& ab : zone_ab) par.push_back(ab[i]);
    for (int j = 0; j < opt.n_zones; ++j) par.push_back(j < static_cast<int>(order.size()) ? 1.0 : 0.0);
    par.push_back(road.half_width - 0.5 * body.width - limit("edge_margin"));
    par.push_back(std::tan(limit("sideslip_max")));

    const auto [fx_min, fx_max] = fx_limits(x[IUX], p);
    for (int k = 0; k < N; ++k) {
        m.lbw[u_at(k) + IFX] = fx_min / p.m;
        m.ubw[u_at(k) + IFX] = fx_max / p.m;
    }

    if (!m.warm) {
        m.x0.assign(n_w(), 0.0);
        for (int k = 0; k <= N; ++k) {
            State guess = z0;
            guess[IX] = s_plan[k];
            if (k > 0) guess[IPSI] = guess[IUY] = guess[IR] = guess[IDELTA] = 0.0;
            std::copy(guess.data(), guess.data() + NX, m.x0.begin() + x_at(k));
        }
    }

    casadi::DMDict args = {{"p", par}, {"lbx", m.lbw}, {"ubx", m.ubw}, {"lbg", m.lbg}, {"ubg", m.ubg}, {"x0", m.x0}};
    if (m.warm && opt.warm_duals) {
        args["lam_x0"] = m.lam_x0;
        args["lam_g0"] = m.lam_g0;
    }
    casadi::Function& solver = m.warm ? m.warm_solver : m.cold;
    auto cpu_now = [] {
        timespec ts;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
        return ts.tv_sec + 1e-9 * ts.tv_nsec;
    };
    const double cpu_start = cpu_now();
    const auto start = std::chrono::steady_clock::now();
    const casadi::DMDict sol = solver(args);
    solve_times.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    const double cpu = cpu_now() - cpu_start;
    const casadi::Dict st = solver.stats();
    const bool success = st.count("success") && st.at("success").as_bool();
    {
        auto get = [&](const char* k) { return st.count(k) ? st.at(k).to_double() : 0.0; };
        SolveStats ss;
        ss.t = obs.t;
        ss.s0 = s0;
        ss.ux0 = z0[IUX];
        ss.iterations = st.count("iter_count") ? st.at("iter_count").to_int() : 0;
        ss.t_wall = solve_times.back();
        // IPOPT times itself; FATROP does not, so the time around the call stands in
        ss.t_total = st.count("t_wall_total") ? get("t_wall_total") : solve_times.back();
        ss.t_proc = st.count("t_proc_total") ? get("t_proc_total") : cpu;
        ss.t_hess = get("t_wall_nlp_hess_l");
        ss.t_jac = get("t_wall_nlp_jac_g");
        ss.t_grad = get("t_wall_nlp_grad_f");
        ss.t_f = get("t_wall_nlp_f");
        ss.t_g = get("t_wall_nlp_g");
        ss.success = success;
        ss.status = st.count("return_status") && st.at("return_status").is_string() ? st.at("return_status").to_string()
                    : st.count("unified_return_status") ? st.at("unified_return_status").to_string() : "";
        ss.warm = m.warm;
        ss.n_obstacles = n_obstacles;
        ss.directive = directed;
        ss.obj = sol.at("f").scalar();
        const std::vector<double> w = sol.at("x").get_elements();
        for (int k = 0; k < N; ++k)
            for (int i = 0; i < NS; ++i) ss.slack_max[i] = std::max(ss.slack_max[i], w[u_at(k) + NU + i]);
        ss.x0_gap = std::sqrt(x0_gap);
        solve_stats.push_back(ss);
    }
    if (!success) ++failures;
    last_success = success;
    m.x0 = sol.at("x").get_elements();
    m.lam_x0 = sol.at("lam_x").get_elements();
    m.lam_g0 = sol.at("lam_g").get_elements();
    m.t = obs.t;
    m.warm = true;

    const std::vector<double>& w = m.x0;
    plan.resize(N + 1, NX);
    for (int k = 0; k <= N; ++k)
        for (int i = 0; i < NX; ++i) plan(k, i) = w[x_at(k) + i];
    plan_inputs.resize(N, NU);
    for (int k = 0; k < N; ++k) plan_inputs.row(k) << w[u_at(k) + IRATE], p.m * w[u_at(k) + IFX];
    return plan_inputs.row(0).transpose();
}

int step_at(const std::vector<double>& times, double t) {
    const double eps = 1e-9;  // a time on a step's start, give or take the rounding of the summed steps, is in it
    if (t < times.front() - eps) return -1;
    return static_cast<int>(std::upper_bound(times.begin(), times.end(), t + eps) - times.begin()) - 1;
}

}  // namespace tc

namespace tc {

std::vector<std::string> NMPC::set_directive(Directive d) {
    Impl& m = *impl_;
    std::vector<std::string> notes;
    auto note = [&](const std::string& what, double asked, double used) {
        std::ostringstream out;
        out << what << ": " << asked << " -> " << used;
        notes.push_back(out.str());
    };
    auto clamp_all = [&](const char* kind, std::map<std::string, double>& values,
                         const std::map<std::string, Range>& ranges) {
        for (auto& [name, value] : values) {
            const auto it = ranges.find(name);
            if (it == ranges.end()) throw std::invalid_argument(std::string("NMPC directive: unknown ") + kind + " '" + name + "'");
            const Range& r = it->second;
            const double used = std::isfinite(value) ? std::clamp(value, r.lo, r.hi) : r.def;
            if (used != value) note(std::string(kind) + " " + name, value, used);
            value = used;
        }
    };
    clamp_all("weight", d.weights, m.weight_range);
    clamp_all("limit", d.limits, m.limit_range);
    if (!(d.ttl > 0.0) || d.ttl > 60.0) {
        const double used = std::isfinite(d.ttl) ? std::clamp(d.ttl, 0.0, 60.0) : 2.0;
        note("ttl", d.ttl, used);
        d.ttl = used;
    }
    std::vector<Zone> zones;
    for (Zone zone : d.zones) {
        if (!std::isfinite(zone.s) || !std::isfinite(zone.e_y) || !std::isfinite(zone.speed)) {
            notes.push_back("zone with a value that is not finite: left out");
            continue;
        }
        for (double* size : {&zone.length, &zone.width}) {
            const double used = std::isfinite(*size) ? std::clamp(*size, 0.0, 100.0) : 1.0;
            if (used != *size) note("zone size", *size, used);
            *size = used;
        }
        zones.push_back(zone);
    }
    d.zones = std::move(zones);
    if (!d.zones.empty() && opt.n_zones == 0) {
        notes.push_back(std::to_string(d.zones.size()) + " zones left out: the NMPC was built with n_zones = 0");
        d.zones.clear();
    } else if (static_cast<int>(d.zones.size()) > opt.n_zones)
        notes.push_back(std::to_string(d.zones.size()) + " zones: only the " + std::to_string(opt.n_zones) +
                        " nearest are planned with at each solve");
    std::lock_guard<std::mutex> lock(m.mutex);
    m.directive = std::move(d);
    m.directed = true;
    return notes;
}

void NMPC::clear_directive() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->directed = false;
}

bool NMPC::directed() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->directed;
}

Directive NMPC::interpret(const std::vector<Intent>& intents, double t, double ttl) const {
    Directive d;
    d.t = t;
    d.ttl = ttl;
    auto or_default = [](double value, double fallback) { return std::isnan(value) ? fallback : value; };
    for (const Intent& intent : intents) {
        switch (intent.kind) {
            case Intent::keep_lane: d.limits["lane_offset"] = opt.lane_offset; break;
            case Intent::lane_left: d.limits["lane_offset"] = opt.lane_offset + road.lane_width; break;
            case Intent::lane_right: d.limits["lane_offset"] = opt.lane_offset - road.lane_width; break;
            case Intent::set_speed:
                if (std::isnan(intent.value)) throw std::invalid_argument("NMPC: set_speed needs a speed [m/s]");
                d.limits["speed_cap"] = intent.value;
                break;
            case Intent::more_gap: {
                const double k = or_default(intent.value, 2.0);
                d.limits["margin_lat"] = k * opt.margin_lat;
                d.limits["margin_long"] = k * opt.margin_long;
                break;
            }
            case Intent::cautious: {
                const double k = or_default(intent.value, 3.0);
                d.weights["accel"] = k * opt.w_accel;
                d.weights["rate"] = k * opt.w_rate;
                d.limits["sideslip_max"] = 0.5 * opt.sideslip_max;
                break;
            }
            case Intent::avoid: d.zones.push_back(intent.zone); break;
        }
    }
    return d;
}

const std::map<std::string, NMPC::Range>& NMPC::weight_ranges() const { return impl_->weight_range; }
const std::map<std::string, NMPC::Range>& NMPC::limit_ranges() const { return impl_->limit_range; }

Intent::Kind intent_kind(const std::string& name) {
    static const std::map<std::string, Intent::Kind> kinds = {
        {"keep_lane", Intent::keep_lane}, {"lane_left", Intent::lane_left}, {"lane_right", Intent::lane_right},
        {"set_speed", Intent::set_speed}, {"more_gap", Intent::more_gap},   {"cautious", Intent::cautious},
        {"avoid", Intent::avoid},
    };
    const auto it = kinds.find(name);
    if (it == kinds.end()) throw std::invalid_argument("unknown intent '" + name + "'");
    return it->second;
}

}  // namespace tc
