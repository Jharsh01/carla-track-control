// Bicycle plant: the bicycle model plus effects the controller model leaves out.
//
// On top of `dynamics` the plant adds
// - actuator saturation (steering angle and rate, tire and engine force limits),
// - a first-order steering actuator lag,
// - first-order tire force build-up (relaxation length),
// and it can be given parameters that differ from the controller's.
#pragma once

#include "sim/plant/interface.hpp"

namespace sim {

class BicyclePlant : public PlantModel {
public:
    // `tau_steer` [s] and `relaxation_length` [m] of 0 switch the respective lag off.
    explicit BicyclePlant(const VehicleParams& params, double dt = 1e-3, double tau_steer = 0.05,
                          double relaxation_length = 0.3);

    void reset(const State& x0) override;
    State observe() const override { return z_.head<NX>(); }
    Input saturate(const Input& u) const override;
    State step(const Input& u, double duration) override;
    void set_friction(double mu) override { p.mu = mu; }

    VehicleParams p;
    double dt, tau_steer, relaxation_length;

private:
    // internal state: model state, steering reference, lagged slip angles
    using Internal = Eigen::Matrix<double, NX + 3, 1>;
    Internal deriv(const Internal& z, const Input& u) const;
    Internal z_;
};

}  // namespace sim
