// Bicycle model: steady cornering against the understeer-gradient formula, force limits, parameter files.
#include <cmath>
#include <cstdio>
#include <fstream>

#include "check.hpp"
#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"

using namespace tc;

// Drive at constant speed and steering angle until the yaw rate settles.
static double steady_yaw_rate(const VehicleParams& p, double speed, double delta) {
    State x = State::Zero();
    x[IUX] = speed;
    x[IDELTA] = delta;
    for (int i = 0; i < 20000; ++i) {  // 20 s
        const double drag = 0.5 * p.rho * p.CdA * x[IUX] * x[IUX] + p.Crr * p.m * p.g;
        // hold the speed with a proportional drive force
        const Input u(0.0, drag + 2.0 * p.m * (speed - x[IUX]));
        x = rk4_step([&](const State& z, const Input& v) { return dynamics(z, v, p); }, x, u, 1e-3);
    }
    return x[IR];
}

// Steady yaw rate against the understeer-gradient formula, at the axle loads of the force holding the speed.
static void understeer_formula(const VehicleParams& p) {
    for (double speed : {10.0, 20.0}) {
        const double delta = 0.01;
        const double K = p.understeer_gradient(0.5 * p.rho * p.CdA * speed * speed + p.Crr * p.m * p.g);
        const double expected = speed * delta / (p.L() + K * speed * speed);
        CHECK_NEAR(steady_yaw_rate(p, speed, delta), expected, 0.01 * std::fabs(expected));
    }
}

// Cornering stiffness against load: through the table's points, monotone between them, flat outside, the same on the model's
// template path as on LoadTable; braking loads the front and stiffens it.
static void load_table() {
    VehicleParams p;
    p.C_alpha_f_load = {{5000.0, 10000.0, 20000.0}, {50000.0, 100000.0, 150000.0}};
    CHECK_NEAR(p.C_alpha_f_at(10000.0), 100000.0, 1e-9);  // through the points
    CHECK(p.C_alpha_f_at(7500.0) > 50000.0 && p.C_alpha_f_at(7500.0) < 100000.0);  // monotone between them
    CHECK(p.C_alpha_f_at(15000.0) > 100000.0 && p.C_alpha_f_at(15000.0) < 150000.0);
    const double eps = 1e-3;  // flat at the end points: no kink into the held values
    CHECK_NEAR((p.C_alpha_f_at(20000.0 + eps) - p.C_alpha_f_at(20000.0 - eps)) / (2 * eps), 0.0, 1e-3);
    CHECK_NEAR(p.C_alpha_f_at(1000.0), 50000.0, 0.0);
    CHECK_NEAR(p.C_alpha_f_at(30000.0), 150000.0, 0.0);
    CHECK_NEAR(p.C_alpha_r_at(1000.0), p.C_alpha_r, 0.0);  // no rear table: the constant
    for (double Fz : {1000.0, 5000.0, 7500.0, 10000.0, 12345.0, 20000.0, 30000.0})
        CHECK_NEAR(cornering_stiffness(p.C_alpha_f_load, p.C_alpha_f, Fz), p.C_alpha_f_at(Fz), 1e-9);
    for (double Fz : {9000.0, 10000.0, 12345.0})  // only the pieces over 9000..13000 built: the same in that range
        CHECK_NEAR(cornering_stiffness(p.C_alpha_f_load, p.C_alpha_f, Fz, 9000.0, 13000.0), p.C_alpha_f_at(Fz), 1e-9);

    State x = State::Zero();
    x[IUX] = 20.0;
    const auto coast = axle_forces(x, Input(0.0, 0.0), p);
    const auto brake = axle_forces(x, Input(0.0, -5000.0), p);
    CHECK(brake.Fzf > coast.Fzf && brake.Fzr < coast.Fzr);
}

static void force_limits() {
    const VehicleParams p;
    const auto [lo, hi] = fx_limits(10.0, p);
    CHECK(lo < 0.0 && hi > 0.0);
    CHECK(-lo <= p.F_brake_max + 1e-9);
    CHECK(hi <= p.F_drive_max + 1e-9);
    CHECK_NEAR(fx_limits(1000.0, p).second, p.P_max / 1000.0, 1e-9);  // power-limited at high speed
}

static void parameter_file() {
    const std::string path = "test_model_params.json";
    {
        std::ofstream out(path);
        out << R"({"vehicle": {"parameters": {)";
        bool first = true;
        for (const auto& field : VehicleParams::fields()) {
            out << (first ? "" : ",") << '"' << field.name << R"(": {"value": )"
                << (std::string(field.name) == "m" ? 1500.0 : VehicleParams().*field.member);
            if (std::string(field.name) == "C_alpha_r") out << R"(, "load_table": {"Fz": [4000, 8000], "C_alpha": [40000, 70000]})";
            out << "}";
            first = false;
        }
        out << "}}}";
    }
    const VehicleParams p = VehicleParams::from_file(path);
    CHECK_NEAR(p.m, 1500.0, 0.0);
    CHECK_NEAR(p.a, VehicleParams().a, 0.0);
    CHECK(p.C_alpha_f_load.empty());
    CHECK_NEAR(p.C_alpha_r_at(4000.0), 40000.0, 1e-9);
    CHECK(p.C_alpha_r_at(6000.0) > 40000.0 && p.C_alpha_r_at(6000.0) < 70000.0);
    std::remove(path.c_str());
}

// Share of each axle's lateral capacity in use.
static std::array<double, 2> grip_used(const State& x, const Input& u, const VehicleParams& p) {
    const auto f = axle_forces(x, u, p);
    return {std::fabs(f.Fyf) / lateral_capacity(f.Fzf, f.Fxf, p.mu_f()),
            std::fabs(f.Fyr) / lateral_capacity(f.Fzr, f.Fxr, p.mu_r())};
}

// From a steady corner (20 m/s, 6 m/s^2), hold the steering and apply Fx for 2 s; returns the peak sideslip [rad].
// Full brake unloads the rear axle and spends half of its friction on braking: the rear saturates first and the car
// oversteers (sideslip and yaw rate grow). Full drive does the same to the front: it understeers (sideslip stays small,
// the yaw rate falls).
static double limit_handling(const VehicleParams& p, double Fx, bool rear_first, const char* name) {
    const double speed = 20.0, delta = 6.0 * p.L() / (speed * speed);
    State x = State::Zero();
    x[IUX] = speed;
    x[IDELTA] = delta;
    auto f = [&](const State& z, const Input& v) { return dynamics(z, v, p); };
    for (int i = 0; i < 10000; ++i) {  // settle the corner, holding the speed
        const double drag = 0.5 * p.rho * p.CdA * x[IUX] * x[IUX] + p.Crr * p.m * p.g;
        x = rk4_step(f, x, Input(0.0, drag + 2.0 * p.m * (speed - x[IUX])), 1e-3);
    }
    const double r0 = x[IR];
    const Input u(0.0, Fx);
    double beta_max = 0.0, first_front = INFINITY, first_rear = INFINITY;
    std::printf("%s from a steady corner, Fx %.0f N:\n     t   beta[deg]  r[deg/s]  Ux[m/s]  front grip  rear grip\n",
                name, Fx);
    for (int i = 0; i <= 2000; ++i) {
        const auto used = grip_used(x, u, p);
        const double t = 1e-3 * i;
        if (used[0] > 0.99) first_front = std::min(first_front, t);
        if (used[1] > 0.99) first_rear = std::min(first_rear, t);
        const double beta = std::atan2(x[IUY], x[IUX]);
        beta_max = std::max(beta_max, std::fabs(beta));
        if (i % 250 == 0)
            std::printf("  %4.2f  %9.2f  %8.2f  %7.2f  %10.2f  %9.2f\n", t, beta * 180.0 / M_PI, x[IR] * 180.0 / M_PI,
                        x[IUX], used[0], used[1]);
        x = rk4_step(f, x, u, 1e-3);
    }
    if (rear_first) {
        CHECK(first_rear < first_front);
        CHECK(beta_max > 10.0 * M_PI / 180.0);
    } else {
        CHECK(first_front < first_rear);
        CHECK(beta_max < 5.0 * M_PI / 180.0);  // the steady corner itself has about 3 deg
        CHECK(x[IR] < r0);
    }
    return beta_max;
}

int main() {
    {
        const VehicleParams p;
        limit_handling(p, fx_limits(20.0, p).first, true, "oversteer, full brake");
        limit_handling(p, fx_limits(20.0, p).second, false, "understeer, full drive");
    }
    understeer_formula(VehicleParams());
#ifdef TEST_VEHICLE_FILE  // set by the repository's build: config/vehicles/model3.json, with load tables
    const VehicleParams model3 = VehicleParams::from_file(TEST_VEHICLE_FILE);
    CHECK(!model3.C_alpha_f_load.empty() && !model3.C_alpha_r_load.empty());
    CHECK_NEAR(model3.C_alpha_f_at(model3.m * model3.g * model3.b / model3.L()), model3.C_alpha_f, 0.01 * model3.C_alpha_f);
    CHECK_NEAR(model3.C_alpha_r_at(model3.m * model3.g * model3.a / model3.L()), model3.C_alpha_r, 0.01 * model3.C_alpha_r);
    understeer_formula(model3);
    std::printf("model3 understeer gradient [deg/g]: full brake %+.3f, coast %+.3f, full drive %+.3f\n",
                model3.understeer_gradient(fx_limits(20.0, model3).first) * model3.g * 180.0 / M_PI,
                model3.understeer_gradient(0.0) * model3.g * 180.0 / M_PI,
                model3.understeer_gradient(fx_limits(20.0, model3).second) * model3.g * 180.0 / M_PI);
#endif
    load_table();
    force_limits();
    parameter_file();
    return check_report("test_model");
}
