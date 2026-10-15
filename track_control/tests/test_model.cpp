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

static void understeer_formula() {
    const VehicleParams p;
    const double K = p.understeer_gradient();
    for (double speed : {10.0, 20.0}) {
        const double delta = 0.01;
        const double expected = speed * delta / (p.L() + K * speed * speed);
        CHECK_NEAR(steady_yaw_rate(p, speed, delta), expected, 0.01 * std::fabs(expected));
    }
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
                << (std::string(field.name) == "m" ? 1500.0 : VehicleParams().*field.member) << "}";
            first = false;
        }
        out << "}}}";
    }
    const VehicleParams p = VehicleParams::from_file(path);
    CHECK_NEAR(p.m, 1500.0, 0.0);
    CHECK_NEAR(p.a, VehicleParams().a, 0.0);
    std::remove(path.c_str());
}

int main() {
    understeer_formula();
    force_limits();
    parameter_file();
    return check_report("test_model");
}
