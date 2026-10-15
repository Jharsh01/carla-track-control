#include "sim/plant/bicycle_plant.hpp"

#include <algorithm>
#include <cmath>

#include "track_control/model/bicycle.hpp"
#include "sim/plant/filters.hpp"
#include "track_control/model/integrate.hpp"

namespace sim {

BicyclePlant::BicyclePlant(const VehicleParams& params, double dt, double tau_steer, double relaxation_length)
    : p(params), dt(dt), tau_steer(tau_steer), relaxation_length(relaxation_length) {
    reset(State::Zero());
}

void BicyclePlant::reset(const State& x0) {
    const auto alpha = slip_angles(x0, p);
    z_ << x0, x0[IDELTA], alpha[0], alpha[1];
}

Input BicyclePlant::saturate(const Input& u) const {
    const auto [fx_min, fx_max] = fx_limits(z_[IUX], p);
    return {std::clamp(u[IRATE], -p.delta_rate_max, p.delta_rate_max), std::clamp(u[IFX], fx_min, fx_max)};
}

BicyclePlant::Internal BicyclePlant::deriv(const Internal& z, const Input& u) const {
    const State x = z.head<NX>();
    const double delta_ref = z[NX];
    const std::array<double, 2> alpha_lag = {z[NX + 1], z[NX + 2]};

    double ddelta_ref = u[IRATE];
    if (std::fabs(delta_ref) >= p.delta_max && ddelta_ref * delta_ref > 0.0) ddelta_ref = 0.0;
    const double ddelta = tau_steer > 0.0 ? first_order_lag(x[IDELTA], delta_ref, tau_steer) : ddelta_ref;

    const std::array<double, 2> alpha_ss = slip_angles(x, p);
    std::array<double, 2> alpha = alpha_ss, dalpha = {0.0, 0.0};
    if (relaxation_length > 0.0) {
        alpha = alpha_lag;
        const double speed = std::max(x[IUX], p.Ux_min);
        for (int i = 0; i < 2; ++i) dalpha[i] = relaxation(alpha_lag[i], alpha_ss[i], speed, relaxation_length);
    }

    const StateOf<double> dx = dynamics<double>(to_array(x), {ddelta, u[IFX]}, p, &alpha);
    Internal dz;
    dz << to_state(dx), ddelta_ref, dalpha[0], dalpha[1];
    return dz;
}

State BicyclePlant::step(const Input& u, double duration) {
    const int steps = static_cast<int>(std::lround(duration / dt));
    for (int i = 0; i < steps; ++i)
        z_ = rk4_step([this](const Internal& z, const Input& v) { return deriv(z, v); }, z_, saturate(u), dt);
    return observe();
}

}  // namespace sim
