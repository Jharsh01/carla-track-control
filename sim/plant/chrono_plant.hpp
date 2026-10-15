// Plant backed by a Project Chrono vehicle: the multibody sedan from Chrono's
// model library on Pacejka (PAC2002) tires.
//
// What Chrono simulates: the chassis in 3D (roll, pitch, heave), double-wishbone
// front and multi-link rear suspension, rack-and-pinion steering, the driveline
// with its differential, the brakes, each wheel's spin, and the four tires with
// their own loads, so load transfers left to right as well as front to rear.
//
// What this class adds to fit the project's interface:
// - input: the steering rate is integrated to a road-wheel angle, passed through
//   a first-order actuator lag and converted to Chrono's steering input; a
//   positive Fx becomes a torque on the driven (front) axle shafts and a negative
//   one brake torques, split between the axles by the parameters' brake_front
//   (0.5: equal torques on all four wheels, as Chrono's own brake input gives). Chrono's engine and gearbox are left out: the drive
//   force is delivered as asked, within the limits in the vehicle parameters.
// - state: [X, Y, psi, Ux, Uy, r, delta] is read at the vehicle's centre of
//   mass; delta is the commanded road-wheel angle after the actuator, as a
//   steering-column sensor would report it. The wheels' true angle differs a
//   little under load (suspension compliance, roll steer).
// - friction: the ground is flat with one friction coefficient, set per step.
//   Chrono scales the tire's own peak friction by mu / 0.8.
//
// reset() places the car at X, Y, psi with speed Ux; Uy, r and delta start at 0.
#pragma once

#include <memory>

#include "sim/plant/interface.hpp"

namespace sim {

class ChronoPlant : public PlantModel {
public:
    // `params` give the actuator limits used by saturate() (steering rate and angle,
    // peak brake and drive force, power) and the drag area.
    explicit ChronoPlant(const VehicleParams& params, double dt = 1e-3, double tau_steer = 0.05);
    ~ChronoPlant() override;

    void reset(const State& x0) override;
    State observe() const override;
    Input saturate(const Input& u) const override;
    State step(const Input& u, double duration) override;
    void set_friction(double mu) override;

    // Properties of the Chrono vehicle, measured on the model at rest.
    struct Facts {
        double mass;  // [kg]
        double Izz;  // yaw inertia about the centre of mass [kg m^2]
        double a, b;  // centre of mass to the front and rear axle [m]
        double h;  // centre of mass height [m]
        double tire_radius;  // [m]
        double steer_gain;  // road-wheel angle per unit of Chrono steering input [rad]
        double brake_force_max;  // all four brakes at their peak torque [N]
    };
    Facts facts() const;

    // Body motion the planar state leaves out, for drawing the car: roll and pitch
    // of the chassis [rad] (roll positive leaning right, pitch positive nose down)
    // and the rise of the centre of mass above its height at rest [m].
    struct Attitude {
        double roll, pitch, heave;
    };
    Attitude attitude() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sim
