// Identify the bicycle-model parameters of the Chrono sedan and write config/vehicles/chrono_sedan.json.
//
// Mass, inertia and geometry are read from the Chrono model. The cornering
// stiffnesses, the friction coefficient and the rolling resistance come from
// manoeuvres driven in Chrono, as they would on a test track:
//   - steady cornering at 0.1 g: axle forces and slip angles give C_alpha front and rear,
//   - steady cornering at growing steering angles on a slippery surface, where the car
//     settles at its limit: the lateral acceleration there gives mu,
//   - coasting: the deceleration left after aerodynamic drag gives Crr.
// The drive limits and the drag area are choices, since Chrono's engine is not used
// and its sedan has no aerodynamic data.
//
// Run scripts/fit_chrono_tires.py afterwards: it replaces the tire values written
// here (one friction coefficient for both axles) with a fit per axle, which the
// NMPC needs to drive the Chrono car at the limit (README).
//
//   identify_chrono [output.json]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>

#include "sim/plant/bicycle_plant.hpp"
#include "sim/plant/chrono_plant.hpp"

using namespace sim;
using json = nlohmann::ordered_json;

namespace {

constexpr double DT = 0.02;

// Drive `plant` for `duration` at `speed`, steering toward `delta(t)`; `sample(t, x)` sees every step.
void drive(PlantModel& plant, const VehicleParams& p, double speed, double duration,
           const std::function<double(double)>& delta, const std::function<void(double, const State&)>& sample) {
    State x0 = State::Zero();
    x0[IUX] = speed;
    plant.reset(x0);
    State x = plant.observe();
    for (double t = 0.0; t < duration; t += DT) {
        const double rate = (delta(t) - x[IDELTA]) / 0.1;
        const double drag = 0.5 * p.rho * p.CdA * x[IUX] * x[IUX] + p.Crr * p.m * p.g;
        x = plant.step({rate, p.m * 2.0 * (speed - x[IUX]) + drag}, DT);
        sample(t + DT, x);
    }
}

struct Steady {
    double Ux, Uy, r, delta;
    double r_spread;  // max - min of the yaw rate over the averaged second
};

// Mean state over the last second of holding a steering angle.
Steady hold(PlantModel& plant, const VehicleParams& p, double speed, double delta, double duration = 8.0) {
    Steady mean{0.0, 0.0, 0.0, 0.0, 0.0};
    double r_min = INFINITY, r_max = -INFINITY;
    int n = 0;
    drive(plant, p, speed, duration, [&](double) { return delta; }, [&](double t, const State& x) {
        if (t <= duration - 1.0) return;
        mean.Ux += x[IUX], mean.Uy += x[IUY], mean.r += x[IR], mean.delta += x[IDELTA];
        r_min = std::min(r_min, x[IR]), r_max = std::max(r_max, x[IR]);
        ++n;
    });
    return {mean.Ux / n, mean.Uy / n, mean.r / n, mean.delta / n, r_max - r_min};
}

json entry(double value, const char* unit, const char* source, const char* description) {
    return {{"value", value}, {"unit", unit}, {"source", source}, {"description", description}};
}

}  // namespace

int main(int argc, char** argv) {
    const std::string output = argc > 1 ? argv[1] : std::string(SIM_CONFIG_DIR "/vehicles/chrono_sedan.json");

    // start from the default set for the constants, with limits wide enough not to interfere
    VehicleParams p;
    p.CdA = 0.6;  // a typical sedan: Cd 0.28 on 2.2 m^2
    p.Crr = 0.0;
    p.drive_front = 1.0;  // Chrono's sedan drives the front axle
    p.brake_front = 0.5;  // the same brake on every wheel
    p.F_drive_max = 6000.0;
    p.P_max = 150e3;
    p.F_brake_max = 1e9;
    p.delta_max = 1.0;
    p.delta_rate_max = 0.6;

    ChronoPlant chrono(p);
    const ChronoPlant::Facts f = chrono.facts();
    p.m = f.mass, p.Izz = f.Izz, p.a = f.a, p.b = f.b, p.h = f.h;
    p.F_brake_max = f.brake_force_max;
    p.delta_max = f.steer_gain;  // Chrono's steering input at its limit
    std::printf("Chrono sedan: m %.0f kg, Izz %.0f kg m^2, a %.3f m, b %.3f m, h %.3f m, tire radius %.3f m\n", p.m,
                p.Izz, p.a, p.b, p.h, f.tire_radius);

    // rolling resistance: coast at 10 m/s
    {
        State x0 = State::Zero();
        x0[IUX] = 10.0;
        chrono.reset(x0);
        chrono.step(Input::Zero(), 1.0);
        const double v1 = chrono.observe()[IUX];
        const double v2 = chrono.step(Input::Zero(), 2.0)[IUX];
        const double decel = (v1 - v2) / 2.0, v = 0.5 * (v1 + v2);
        p.Crr = std::max(0.0, (p.m * decel - 0.5 * p.rho * p.CdA * v * v) / (p.m * p.g));
        std::printf("coasting at %.1f m/s: %.3f m/s^2, Crr %.4f\n", v, decel, p.Crr);
    }

    // cornering stiffness: steady cornering at about 0.1 g
    {
        const double speed = 15.0, delta = 0.1 * p.g * p.L() / (speed * speed);
        const Steady s = hold(chrono, p, speed, delta);
        const double ay = s.r * s.Ux;
        const double alpha_f = std::atan((s.Uy + p.a * s.r) / s.Ux) - s.delta;
        const double alpha_r = std::atan((s.Uy - p.b * s.r) / s.Ux);
        p.C_alpha_f = -p.m * ay * p.b / p.L() / alpha_f;
        p.C_alpha_r = -p.m * ay * p.a / p.L() / alpha_r;
        std::printf("steady cornering at %.2f m/s^2: slip %.4f / %.4f rad, C_alpha %.0f / %.0f N/rad (front / rear)\n",
                    ay, alpha_f, alpha_r, p.C_alpha_f, p.C_alpha_r);
    }

    // friction: on a slippery surface the car settles at its cornering limit instead of spinning. Chrono
    // scales the tire's grip in proportion to the ground coefficient, so the result carries over to the dry road.
    {
        const double speed = 15.0, slippery = 0.5, dry = 0.9;
        chrono.set_friction(slippery);
        double ay_limit = 0.0;
        for (double delta = 0.10; delta <= 0.26; delta += 0.04) {
            const Steady c = hold(chrono, p, speed, delta, 10.0);
            if (c.r_spread < 0.01) ay_limit = std::max(ay_limit, c.r * c.Ux);
        }
        chrono.set_friction(dry);
        p.mu = ay_limit / p.g * dry / slippery;
        std::printf("cornering limit on ground coefficient %.1f: %.2f m/s^2 (%.2f g); mu on the dry road (%.1f): %.3f\n",
                    slippery, ay_limit, ay_limit / p.g, dry, p.mu);
    }

    // how well the bicycle model with these parameters reproduces Chrono
    std::printf("\nsteady yaw rate, Chrono against the bicycle model with the identified parameters:\n");
    BicyclePlant bicycle(p);
    for (const auto& [speed, g_level] : {std::pair{15.0, 0.2}, {15.0, 0.4}, {15.0, 0.6}, {22.0, 0.2}, {22.0, 0.4}, {22.0, 0.6}}) {
        const double delta = g_level * p.g * (p.L() + p.understeer_gradient() * speed * speed) / (speed * speed);
        const Steady c = hold(chrono, p, speed, delta), b = hold(bicycle, p, speed, delta);
        std::printf("  %4.0f m/s, steer %.4f rad: Chrono %.4f rad/s (%.2f m/s^2), bicycle %.4f rad/s, difference %+.1f %%\n",
                    speed, delta, c.r, c.r * c.Ux, b.r, 100.0 * (b.r / c.r - 1.0));
    }

    // write the file in the layout of model3.json
    std::ifstream base_file(SIM_CONFIG_DIR "/vehicles/model3.json");
    const json base = json::parse(base_file);
    json out;
    out["state"] = base.at("state");
    out["input"] = base.at("input");
    out["vehicle"]["description"] =
        "Project Chrono sedan (chrono_models Sedan_Vehicle) on PAC2002 tires 245/40 R18; written by identify_chrono";
    out["vehicle"]["sources"] = {
        {"chrono", "read from the Chrono model at rest"},
        {"identified", "from a manoeuvre driven in Chrono (see sim/plant/identify_chrono.cpp)"},
        {"design", "chosen for this project"},
        {"constant", "physical constant"},
    };
    json& q = out["vehicle"]["parameters"];
    q["m"] = entry(p.m, "kg", "chrono", "mass");
    q["Izz"] = entry(p.Izz, "kg m^2", "chrono", "yaw inertia about the centre of mass");
    q["a"] = entry(p.a, "m", "chrono", "centre of mass to front axle");
    q["b"] = entry(p.b, "m", "chrono", "centre of mass to rear axle");
    q["h"] = entry(p.h, "m", "chrono", "centre of mass height");
    q["C_alpha_f"] = entry(p.C_alpha_f, "N/rad", "identified", "front axle cornering stiffness, steady cornering at 0.1 g");
    q["C_alpha_r"] = entry(p.C_alpha_r, "N/rad", "identified", "rear axle cornering stiffness, steady cornering at 0.1 g");
    q["mu"] = entry(p.mu, "-", "identified", "cornering limit over g on ground of coefficient 0.9 (the dry road)");
    q["drive_front"] = entry(p.drive_front, "-", "chrono", "share of drive force on the front axle (front-wheel drive)");
    q["brake_front"] = entry(p.brake_front, "-", "chrono", "share of brake force on the front axle (equal brake torques)");
    q["F_drive_max"] = entry(p.F_drive_max, "N", "design", "peak drive force; Chrono's engine is not used");
    q["F_brake_max"] = entry(p.F_brake_max, "N", "chrono", "four brakes at 2000 N m over the tire radius");
    q["P_max"] = entry(p.P_max, "W", "design", "drive power; Chrono's engine is not used");
    q["CdA"] = entry(p.CdA, "m^2", "design", "drag coefficient times frontal area, applied to the Chrono chassis");
    q["Crr"] = entry(p.Crr, "-", "identified", "rolling resistance coefficient, from coasting");
    q["rho"] = entry(p.rho, "kg/m^3", "constant", "air density");
    q["delta_max"] = entry(p.delta_max, "rad", "chrono", "road-wheel angle at full steering input");
    q["delta_rate_max"] = entry(p.delta_rate_max, "rad/s", "design", "road-wheel steering rate limit");
    q["Ux_min"] = entry(p.Ux_min, "m/s", "design", "slip-angle denominator guard; the model is meant for Ux > 3");
    q["g"] = entry(p.g, "m/s^2", "constant", "gravity");
    std::ofstream(output) << out.dump(2) << '\n';
    std::printf("\nwrote %s\n", output.c_str());
    return 0;
}
