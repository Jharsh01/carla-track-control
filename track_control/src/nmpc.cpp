#include "track_control/nmpc.hpp"

#include <algorithm>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>

#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"

namespace tc {

using casadi::Slice;
using casadi::SX;
static constexpr double FAR = 1e4;  // s offset that puts an unused obstacle slot out of reach [m]

namespace {

StateOf<SX> state_of(const SX& x) { return {x(IX), x(IY), x(IPSI), x(IUX), x(IUY), x(IR), x(IDELTA)}; }

double tightest_corner_speed(const Path& path, const Road& road, const VehicleParams& p) {
    const double kappa_max = path.max_curvature();
    return kappa_max > 0.0 ? std::sqrt(road.mu * p.g / kappa_max) : INF;
}

}  // namespace

struct NMPC::Impl {
    casadi::Function solver;
    std::vector<double> lbw, ubw, lbg, ubg;
    // previous primal and dual solution, to start the next solve from
    std::vector<double> x0;
    casadi::DM lam_x0, lam_g0;
    bool warm = false;
};

NMPC::NMPC(Path path_, Road road_, VehicleParams params, Body body_, Options o)
    : path(std::move(path_)),
      road(road_),
      p(params),
      body(body_),
      opt(o),
      h(o.h),
      N(static_cast<int>(std::lround(o.horizon / o.h))),
      unseen_speed(std::min(o.unseen_speed.value_or(tightest_corner_speed(path, road_, params)), road_.v_max)),
      impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    const int n_cars = o.n_cars;
    // ellipse around an obstacle, in ego centre-of-gravity coordinates
    const double ly = body.width + o.margin_lat;
    const double lx = body.length + o.margin_long;
    const double B = road.lane_width - 0.4;
    const double A = lx / std::sqrt(1.0 - (ly / B) * (ly / B));
    const double e_max = road.half_width - 0.5 * body.width - o.edge_margin;
    const double to_front = 0.5 * body.length - body.cg_to_center;
    const double to_rear = 0.5 * body.length + body.cg_to_center;
    const double slip = std::tan(o.sideslip_max);
    const double e_ref = o.lane_offset;

    // parameters: measured state, curvature and speed limit of each step,
    // obstacle positions at each predicted state
    const SX x_init = SX::sym("x_init", NX);
    const SX kappa = SX::sym("kappa", N);
    const SX v_lim = SX::sym("v_lim", N);
    const SX car_s = SX::sym("car_s", n_cars, N);
    const SX car_e = SX::sym("car_e", n_cars);
    const SX par = SX::vertcat({x_init, kappa, v_lim, vec(car_s), car_e});

    const SX x = SX::sym("x", NX, N + 1);
    const SX u = SX::sym("u", NU, N);  // the longitudinal input is the acceleration Fx / m, for scaling
    const SX slack = SX::sym("s", NS, N);

    const VehicleParams& pp = p;
    SX J = 0.0;
    std::vector<SX> g_eq = {x(Slice(), 0) - x_init};
    std::vector<SX> g_in;  // each entry <= 0
    for (int k = 0; k < N; ++k) {
        const SX xk = x(Slice(), k), uk = u(Slice(), k), nxt = x(Slice(), k + 1), sk = slack(Slice(), k);
        const SX kap = kappa(k);
        auto f = [&pp, &kap](const SX& z, const SX& v) {
            // with psi = e_psi
            const StateOf<SX> d = dynamics<SX>(state_of(z), {v(IRATE), pp.m * v(IFX)}, pp);
            const SX s_dot = d[IX] / (1.0 - kap * z(IY));
            return SX::vertcat({s_dot, d[IY], d[IPSI] - kap * s_dot, d[IUX], d[IUY], d[IR], d[IDELTA]});
        };
        g_eq.push_back(nxt - rk4_step(f, xk, uk, h));

        const SX v_lat = nxt(IUX) * sin(nxt(IPSI)) + nxt(IUY) * cos(nxt(IPSI));
        J += o.w_speed * sq(nxt(IUX) - road.v_max) + o.w_lane * sq(nxt(IY) - e_ref) +
             o.w_lat * (sq(v_lat) + sq(nxt(IR) - kap * nxt(IUX))) + o.w_accel * sq(uk(IFX)) +
             o.w_rate * sq(uk(IRATE)) + o.w_slack * (sum1(sk) + sumsqr(sk));

        for (int j = 0; j < n_cars; ++j) {
            const SX ds = nxt(IX) - body.cg_to_center - car_s(j, k);
            g_in.push_back(1.0 - sq(ds / A) - sq((nxt(IY) - car_e(j)) / B) - sk(0));
        }
        for (double arm : {to_front, -to_rear}) {  // road edges, at the front and rear ends of the body
            const SX e_end = nxt(IY) + arm * sin(nxt(IPSI));
            g_in.push_back(e_end - e_max - sk(1));
            g_in.push_back(-e_max - e_end - sk(1));
        }
        g_in.push_back(nxt(IUY) - slip * nxt(IUX) - sk(2));
        g_in.push_back(-nxt(IUY) - slip * nxt(IUX) - sk(2));
        g_in.push_back(nxt(IUX) - v_lim(k) - sk(3));
    }

    const SX eq = SX::vertcat(g_eq), in = SX::vertcat(g_in);
    m.lbg.assign(eq.size1(), 0.0);
    m.lbg.insert(m.lbg.end(), in.size1(), -INF);
    m.ubg.assign(eq.size1() + in.size1(), 0.0);

    for (int k = 0; k < N; ++k) {  // force limits are set per solve
        m.lbw.insert(m.lbw.end(), {-p.delta_rate_max, 0.0});
        m.ubw.insert(m.ubw.end(), {p.delta_rate_max, 0.0});
    }
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < NX; ++i) {
            const bool bounded = k > 0;
            // the tire model is not meant for speeds below 3 m/s
            m.lbw.push_back(bounded && i == IDELTA ? -p.delta_max : bounded && i == IUX ? 3.0 : -INF);
            m.ubw.push_back(bounded && i == IDELTA ? p.delta_max : INF);
        }
    }
    m.lbw.insert(m.lbw.end(), NS * N, 0.0);
    m.ubw.insert(m.ubw.end(), NS * N, INF);

    const casadi::SXDict nlp = {
        {"x", SX::vertcat({vec(u), vec(x), vec(slack)})}, {"p", par}, {"f", J}, {"g", SX::vertcat({eq, in})}};
    const casadi::Dict solver_options = {
        {"print_time", false},
        {"ipopt.print_level", 0},
        {"ipopt.sb", "yes"},
        {"ipopt.max_iter", o.max_iter},
        {"ipopt.tol", 1e-6},
    };
    m.solver = casadi::nlpsol("track_nmpc", "ipopt", nlp, solver_options);
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

    std::vector<double> s_plan(N + 1);
    for (int k = 0; k <= N; ++k)  // constant speed at first, then the previous plan's progress from where the car is now
        s_plan[k] = s0 + (plan.size() ? plan(k, IX) - plan(0, IX) : z0[IUX] * h * k);

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
        for (int k = 0; k < N; ++k) s[k] = path.offset_to_s(start + car.speed * h * (k + 1), car.e_y);
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
    while (static_cast<int>(slots.size()) < n_cars) slots.push_back({0.0, std::vector<double>(N, s0 + FAR), 0.0});

    std::vector<double> par(z0.data(), z0.data() + NX);
    for (int k = 0; k < N; ++k) {  // mean known curvature over each step
        double sum = 0.0;
        for (int i = 0; i < 5; ++i) {
            const double w = i / 4.0;
            sum += known_curvature(s_plan[k] * (1.0 - w) + s_plan[k + 1] * w);
        }
        par.push_back(sum / 5.0);
    }
    for (int k = 0; k < N; ++k) par.push_back(s_plan[k + 1] <= s0 + opt.road_range ? road.v_max : unseen_speed);
    for (int k = 0; k < N; ++k)
        for (int j = 0; j < n_cars; ++j) par.push_back(slots[j].s[k]);
    for (const Slot& slot : slots) par.push_back(slot.e_y);

    const auto [fx_min, fx_max] = fx_limits(x[IUX], p);
    for (int k = 0; k < N; ++k) {
        m.lbw[NU * k + IFX] = fx_min / p.m;
        m.ubw[NU * k + IFX] = fx_max / p.m;
    }

    if (!m.warm) {
        m.x0.assign(NU * N, 0.0);
        for (int k = 0; k <= N; ++k) {
            State guess = z0;
            guess[IX] = s_plan[k];
            if (k > 0) guess[IPSI] = guess[IUY] = guess[IR] = guess[IDELTA] = 0.0;
            m.x0.insert(m.x0.end(), guess.data(), guess.data() + NX);
        }
        m.x0.insert(m.x0.end(), NS * N, 0.0);
    } else {
        // the warm start's s values belong to the previous lap position; move them to this one
        const double shift = s0 - m.x0[NU * N + IX];
        for (int k = 0; k <= N; ++k) m.x0[NU * N + NX * k + IX] += shift;
    }

    casadi::DMDict args = {{"p", par}, {"lbx", m.lbw}, {"ubx", m.ubw}, {"lbg", m.lbg}, {"ubg", m.ubg}, {"x0", m.x0}};
    if (m.warm) {
        args["lam_x0"] = m.lam_x0;
        args["lam_g0"] = m.lam_g0;
    }
    const auto start = std::chrono::steady_clock::now();
    const casadi::DMDict sol = m.solver(args);
    solve_times.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    if (!m.solver.stats().at("success").as_bool()) ++failures;
    m.x0 = sol.at("x").get_elements();
    m.lam_x0 = sol.at("lam_x");
    m.lam_g0 = sol.at("lam_g");
    m.warm = true;

    const std::vector<double>& w = m.x0;
    plan = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        w.data() + NU * N, N + 1, NX);
    return {w[IRATE], p.m * w[IFX]};
}

}  // namespace tc
