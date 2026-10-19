// Parameters of the single-track model the controllers plan with.
#pragma once

#include <string>
#include <vector>

namespace tc {

// Axle cornering stiffness against axle normal load: C_alpha [N/rad] at Fz [N],
// interpolated by a monotone cubic in between and held at the end values outside
// (model/tire.hpp, cornering_stiffness).
struct LoadTable {
    std::vector<double> Fz, C_alpha;
    bool empty() const { return Fz.empty(); }
    double operator()(double load) const;
};

// The defaults are CARLA 0.9.15's vehicle.tesla.model3 (config/vehicles/model3.json
// in the repository this library ships in, with where each value comes from).
// Cornering stiffnesses are per axle (both tires). With a load table for an axle,
// its stiffness follows the axle's normal load (which shifts with the
// longitudinal load transfer); without one it is the constant C_alpha_f / _r.
struct VehicleParams {
    double m = 1845.0;  // mass [kg]
    double Izz = 3958.0;  // yaw inertia [kg m^2]
    double a = 1.168;  // centre of gravity to front axle [m]
    double b = 1.836;  // centre of gravity to rear axle [m]
    double h = 0.45;  // centre of gravity height [m]
    double C_alpha_f = 106500.0;  // front cornering stiffness [N/rad]
    double C_alpha_r = 67800.0;  // rear cornering stiffness [N/rad]
    double mu = 0.9;  // tire-road friction
    double grip_f = 1.0;  // peak friction of the front tires relative to mu
    double grip_r = 1.0;  // peak friction of the rear tires relative to mu
    double drive_front = 0.5;  // share of the drive force on the front axle
    double brake_front = 0.5;  // share of the brake force on the front axle
    double F_drive_max = 18073.0;  // peak drive force [N]
    double F_brake_max = 7568.0;  // peak brake force [N]
    double P_max = 700000.0;  // drive power [W]
    double CdA = 0.483;  // drag area [m^2]
    double Crr = 0.012;  // rolling resistance coefficient
    double rho = 1.225;  // air density [kg/m^3]
    double delta_max = 0.855;  // road-wheel angle limit [rad]
    double delta_rate_max = 0.6;  // steering rate limit [rad/s]
    double Ux_min = 1.0;  // floor on the speed in the slip-angle denominators [m/s]
    double g = 9.81;  // [m/s^2]
    LoadTable C_alpha_f_load, C_alpha_r_load;  // optional; empty: the constants above

    // Read from a JSON file of the form {"vehicle": {"parameters": {"m": {"value": ...}, ...}}};
    // every field above must be present, except grip_f and grip_r (default 1). An entry of C_alpha_f or C_alpha_r may
    // carry "load_table": {"Fz": [...], "C_alpha": [...]}, Fz increasing; its "value" is then the stiffness at the
    // static load, for reference. The file's "state" and "input" lists, if present, must name the state and input in
    // the library's order.
    static VehicleParams from_file(const std::string& path);

    double L() const { return a + b; }  // wheelbase [m]
    double mu_f() const { return mu * grip_f; }  // peak friction of the front axle
    double mu_r() const { return mu * grip_r; }  // peak friction of the rear axle
    // Cornering stiffness of each axle at its normal load Fz [N]: the table's, or the constant.
    double C_alpha_f_at(double Fz) const { return C_alpha_f_load.empty() ? C_alpha_f : C_alpha_f_load(Fz); }
    double C_alpha_r_at(double Fz) const { return C_alpha_r_load.empty() ? C_alpha_r : C_alpha_r_load(Fz); }
    // Linear range [rad per m/s^2], with the axle loads under a steady longitudinal force Fx [N];
    // positive understeers, negative oversteers.
    double understeer_gradient(double Fx = 0.0) const;

    struct Field {
        const char* name;
        double VehicleParams::* member;
        bool optional = false;
    };
    static const std::vector<Field>& fields();
};

}  // namespace tc
