#include "track_control/model/bicycle.hpp"

#include <algorithm>
#include <limits>

namespace tc {

std::pair<double, double> fx_limits(double Ux, const VehicleParams& p) {
    const double mu_f = p.mu_f(), mu_r = p.mu_r();
    const double grip_f = mu_f * p.m * p.g, grip_r = mu_r * p.m * p.g;
    auto axle_limit = [](double num, double den) {
        return den > 0.0 ? num / den : std::numeric_limits<double>::infinity();
    };
    // each axle's share of Fx within its friction times its load, with the load transfer Fx h / L
    const double fb = p.brake_front, fd = p.drive_front, L = p.L();
    const double brake = std::min({axle_limit(grip_f * p.b, fb * L - mu_f * p.h),
                                   axle_limit(grip_r * p.a, (1.0 - fb) * L + mu_r * p.h), p.F_brake_max});
    const double drive = std::min({axle_limit(grip_f * p.b, fd * L + mu_f * p.h),
                                   axle_limit(grip_r * p.a, (1.0 - fd) * L - mu_r * p.h), p.F_drive_max,
                                   p.P_max / std::max(Ux, p.Ux_min)});
    return {-brake, drive};
}

}  // namespace tc
