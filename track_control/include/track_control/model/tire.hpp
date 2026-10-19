// Fiala brush tire model with friction-circle derating.
#pragma once

#include <cmath>
#include <vector>

#include "track_control/model/math.hpp"
#include "track_control/vehicle_params.hpp"

namespace tc {

constexpr double F_MIN = 50.0;  // floor on lateral capacity [N], keeps the model smooth when Fx uses all the grip

// Lateral force left on the friction circle, sqrt((mu Fz)^2 - Fx^2), smoothed near zero.
template <class T>
T lateral_capacity(const T& Fz, const T& Fx, double mu) {
    using namespace ops;
    T s = (mu * Fz) * (mu * Fz) - Fx * Fx;
    return sqrt(0.5 * (s + sqrt(s * s + 4.0 * F_MIN * F_MIN * F_MIN * F_MIN)));
}

// Axle cornering stiffness [N/rad] at normal load Fz: `table` interpolated by a
// monotone cubic (Fritsch-Carlson), flat at and beyond its end points so the
// stiffness has a continuous derivative everywhere (the NMPC's solver needs it);
// `C_alpha` if the table is empty. Only the pieces overlapping the loads
// [Fz_lo, Fz_hi] are built; beyond those, the end values of the pieces built hold.
template <class T>
T cornering_stiffness(const LoadTable& table, double C_alpha, const T& Fz, double Fz_lo = -INFINITY,
                      double Fz_hi = INFINITY) {
    using namespace ops;
    if (table.empty()) return T(C_alpha);
    const std::vector<double>& x = table.Fz;
    const std::vector<double>& y = table.C_alpha;
    const size_t n = x.size();
    if (n == 1) return T(y[0]);
    // slopes at the points: 0 at the ends, else the harmonic mean of the neighbouring secants (0 at an extremum)
    std::vector<double> d(n, 0.0);
    for (size_t i = 1; i + 1 < n; ++i) {
        const double s0 = (y[i] - y[i - 1]) / (x[i] - x[i - 1]), s1 = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        if (s0 * s1 > 0.0) d[i] = 2.0 / (1.0 / s0 + 1.0 / s1);
    }
    size_t first = 0, last = n - 1;  // the pieces from point `first` to point `last`
    while (first + 1 < n - 1 && x[first + 1] <= Fz_lo) ++first;
    while (last > first + 1 && x[last - 1] >= Fz_hi) --last;
    T C = T(y[first]);
    for (size_t i = first; i < last; ++i) {
        const double h = x[i + 1] - x[i];
        const T t = (Fz - x[i]) / h;
        const T t2 = t * t, t3 = t2 * t;
        const T piece = (2.0 * t3 - 3.0 * t2 + 1.0) * y[i] + (t3 - 2.0 * t2 + t) * (h * d[i])
                        + (-2.0 * t3 + 3.0 * t2) * y[i + 1] + (t3 - t2) * (h * d[i + 1]);
        C = if_else(Fz > x[i], piece, C);
    }
    return if_else(Fz >= x[last], T(y[last]), C);
}

// Axle lateral force [N] for slip angle alpha [rad], normal load Fz and longitudinal force Fx.
template <class T>
T fiala_lateral(const T& alpha, const T& Fz, const T& Fx, const T& C_alpha, double mu) {
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
