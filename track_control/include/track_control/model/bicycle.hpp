// Dynamic single-track (bicycle) model.
//
// State and input as in types.hpp. Fx is split between the axles by the drive
// and brake shares; longitudinal load transfer is quasi-static; the lateral
// tire forces come from the Fiala model with friction-circle derating, each
// axle's cornering stiffness following its load if the parameters give a table.
//
// The equations are templates on the scalar type: double for simulation,
// casadi::SX for the NMPC.
#pragma once

#include <array>
#include <utility>

#include "track_control/model/tire.hpp"
#include "track_control/types.hpp"
#include "track_control/vehicle_params.hpp"

namespace tc {

template <class T>
using StateOf = std::array<T, NX>;
template <class T>
using InputOf = std::array<T, NU>;

// Normal, longitudinal and lateral force on each axle, and the slip angles used.
template <class T>
struct AxleForces {
    T Fzf, Fzr, Fxf, Fxr, Fyf, Fyr, alpha_f, alpha_r;
};

// Front and rear axle slip angles [rad].
template <class T>
std::array<T, 2> slip_angles(const StateOf<T>& x, const VehicleParams& p) {
    using namespace ops;
    T Ux = fmax(x[IUX], T(p.Ux_min));
    return {atan((x[IUY] + p.a * x[IR]) / Ux) - x[IDELTA], atan((x[IUY] - p.b * x[IR]) / Ux)};
}

// `alpha` overrides the kinematic slip angles (a plant with tire lag passes its own).
template <class T>
AxleForces<T> axle_forces(const StateOf<T>& x, const InputOf<T>& u, const VehicleParams& p,
                          const std::array<T, 2>* alpha = nullptr) {
    using namespace ops;
    T Fx = u[IFX];
    T front = if_else(Fx >= 0.0, T(p.drive_front), T(p.brake_front));
    T Fxf = front * Fx;
    T Fxr = Fx - Fxf;
    // quasi-static longitudinal load transfer
    T Fzf = (p.m * p.g * p.b - p.h * Fx) / p.L();
    T Fzr = (p.m * p.g * p.a + p.h * Fx) / p.L();
    std::array<T, 2> a = alpha ? *alpha : slip_angles(x, p);
    // stiffness at the axle loads, the tables' pieces between the loads full brake and full drive give
    const double Fzf_lo = (p.m * p.g * p.b - p.h * p.F_drive_max) / p.L();
    const double Fzf_hi = (p.m * p.g * p.b + p.h * p.F_brake_max) / p.L();
    const double Fzr_lo = (p.m * p.g * p.a - p.h * p.F_brake_max) / p.L();
    const double Fzr_hi = (p.m * p.g * p.a + p.h * p.F_drive_max) / p.L();
    T Cf = cornering_stiffness(p.C_alpha_f_load, p.C_alpha_f, Fzf, Fzf_lo, Fzf_hi);
    T Cr = cornering_stiffness(p.C_alpha_r_load, p.C_alpha_r, Fzr, Fzr_lo, Fzr_hi);
    T Fyf = fiala_lateral(a[0], Fzf, Fxf, Cf, p.mu_f());
    T Fyr = fiala_lateral(a[1], Fzr, Fxr, Cr, p.mu_r());
    return {Fzf, Fzr, Fxf, Fxr, Fyf, Fyr, a[0], a[1]};
}

// Continuous-time state derivative.
template <class T>
StateOf<T> dynamics(const StateOf<T>& x, const InputOf<T>& u, const VehicleParams& p,
                    const std::array<T, 2>* alpha = nullptr) {
    using namespace ops;
    const T& psi = x[IPSI];
    const T& Ux = x[IUX];
    const T& Uy = x[IUY];
    const T& r = x[IR];
    const T& delta = x[IDELTA];
    AxleForces<T> f = axle_forces(x, u, p, alpha);
    T drag = 0.5 * p.rho * p.CdA * (Ux * Ux) + p.Crr * p.m * p.g;
    T Fy_front_body = f.Fyf * cos(delta) + f.Fxf * sin(delta);

    T dUx = (f.Fxf * cos(delta) - f.Fyf * sin(delta) + f.Fxr - drag) / p.m + r * Uy;
    T dUy = (Fy_front_body + f.Fyr) / p.m - r * Ux;
    T dr = (p.a * Fy_front_body - p.b * f.Fyr) / p.Izz;
    T dX = Ux * cos(psi) - Uy * sin(psi);
    T dY = Ux * sin(psi) + Uy * cos(psi);
    return {dX, dY, r, dUx, dUy, dr, u[IRATE]};
}

// The same on the Eigen vector types.
inline StateOf<double> to_array(const State& x) { return {x[0], x[1], x[2], x[3], x[4], x[5], x[6]}; }
inline InputOf<double> to_array(const Input& u) { return {u[0], u[1]}; }
inline State to_state(const StateOf<double>& x) { return State(x.data()); }

inline State dynamics(const State& x, const Input& u, const VehicleParams& p) {
    return to_state(dynamics<double>(to_array(x), to_array(u), p));
}
inline AxleForces<double> axle_forces(const State& x, const Input& u, const VehicleParams& p) {
    return axle_forces<double>(to_array(x), to_array(u), p);
}
inline std::array<double, 2> slip_angles(const State& x, const VehicleParams& p) {
    return slip_angles<double>(to_array(x), p);
}

// Range (min, max) of total longitudinal force [N] the tires, brakes and engine
// can deliver in a straight line. The fixed front/rear split means one axle
// reaches mu*Fz first, so the grip limits sit below mu*m*g.
std::pair<double, double> fx_limits(double Ux, const VehicleParams& p);

}  // namespace tc
