// Fiala brush tire model with friction-circle derating.
#pragma once

#include "track_control/model/math.hpp"

namespace tc {

constexpr double F_MIN = 50.0;  // floor on lateral capacity [N], keeps the model smooth when Fx uses all the grip

// Lateral force left on the friction circle, sqrt((mu Fz)^2 - Fx^2), smoothed near zero.
template <class T>
T lateral_capacity(const T& Fz, const T& Fx, double mu) {
    using namespace ops;
    T s = (mu * Fz) * (mu * Fz) - Fx * Fx;
    return sqrt(0.5 * (s + sqrt(s * s + 4.0 * F_MIN * F_MIN * F_MIN * F_MIN)));
}

// Axle lateral force [N] for slip angle alpha [rad], normal load Fz and longitudinal force Fx.
template <class T>
T fiala_lateral(const T& alpha, const T& Fz, const T& Fx, double C_alpha, double mu) {
    using namespace ops;
    T Fy_max = lateral_capacity(Fz, Fx, mu);
    T z = tan(alpha);
    T z_sl = 3.0 * Fy_max / C_alpha;  // tan of the slip angle at full sliding
    T Fy_grip = -C_alpha * z + C_alpha * C_alpha / (3.0 * Fy_max) * fabs(z) * z
                - C_alpha * C_alpha * C_alpha / (27.0 * Fy_max * Fy_max) * (z * z * z);
    T Fy_slide = -Fy_max * sign(z);
    return if_else(fabs(z) < z_sl, Fy_grip, Fy_slide);
}

}  // namespace tc
