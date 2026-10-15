#include "sim/plant/chrono_plant.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <chrono/core/ChRotation.h>
#include <chrono_models/vehicle/sedan/Sedan_Pac02Tire.h>
#include <chrono_models/vehicle/sedan/Sedan_Vehicle.h>
#include <chrono_vehicle/ChTerrain.h>
#include <chrono_vehicle/ChVehicleModelData.h>

#include "track_control/model/bicycle.hpp"
#include "sim/plant/filters.hpp"

namespace sim {

using chrono::ChVector3d;
using chrono::vehicle::LEFT;
using chrono::vehicle::RIGHT;
using chrono::vehicle::sedan::Sedan_Pac02Tire;
using chrono::vehicle::sedan::Sedan_Vehicle;

namespace {

constexpr double BRAKE_TORQUE_MAX = 2000.0;  // per wheel, Chrono's Sedan_BrakeSimple [N m]
constexpr double GROUND_MU = 0.9;  // ground friction until a scenario sets it

// Flat ground at z = 0 with one friction coefficient.
class Ground : public chrono::vehicle::ChTerrain {
public:
    double GetHeight(const ChVector3d&) const override { return 0.0; }
    ChVector3d GetNormal(const ChVector3d&) const override { return {0.0, 0.0, 1.0}; }
    float GetCoefficientFriction(const ChVector3d&) const override { return static_cast<float>(mu); }
    double mu = 0.8;
};

// Silences std::cerr while in scope: Chrono lists the optional sections missing from the tire file, per tire.
struct Quiet {
    std::streambuf* saved = std::cerr.rdbuf(nullptr);
    ~Quiet() { std::cerr.rdbuf(saved); }
};

}  // namespace

struct ChronoPlant::Impl {
    VehicleParams p;
    double dt, tau_steer;
    Ground ground;
    std::unique_ptr<Sedan_Vehicle> car;
    ChVector3d com{0.0, 0.0, 0.0};  // centre of mass in the vehicle frame
    double ride_height = 0.45;  // height of the vehicle frame with the car at rest [m]
    double steer_gain = 0.5;  // [rad per unit steering input]
    double tire_radius = 0.33;
    std::vector<double> shaft_ratio;  // speed of each driveline shaft per unit of wheel spin, rolling freely
    double delta_ref = 0.0, delta_act = 0.0;  // commanded road-wheel angle, and after the actuator lag
    State x_start = State::Zero();  // the state the car was built in
    double time = 0.0;

    // A fresh car with its centre of mass at (X, Y), heading psi, moving forward at `speed`.
    void build(double X, double Y, double psi, double speed) {
        const Quiet quiet;
        car = std::make_unique<Sedan_Vehicle>(false, chrono::vehicle::BrakeType::SIMPLE, chrono::ChContactMethod::SMC,
                                              chrono::vehicle::CollisionType::NONE);
        const double c = std::cos(psi), s = std::sin(psi);
        const ChVector3d origin(X - (c * com.x() - s * com.y()), Y - (s * com.x() + c * com.y()), ride_height);
        const double spin = speed / tire_radius;
        car->SetInitWheelAngVel({spin, spin, spin, spin});
        car->Initialize(chrono::ChCoordsysd(origin, chrono::QuatFromAngleZ(psi)), speed);
        car->GetChassis()->SetAerodynamicDrag(p.CdA, 1.0, p.rho);
        for (auto& axle : car->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = chrono_types::make_shared<Sedan_Pac02Tire>("tire");
                car->InitializeTire(tire, wheel, chrono::vehicle::VisualizationType::NONE);
                tire->SetStepsize(dt);
            }
        }
        car->InitializeInertiaProperties();
        // Chrono gives the forward speed to the chassis and the spin to the wheels only; set the suspension
        // and the driveline moving with them
        for (auto& body : car->GetSystem()->GetBodies()) body->SetPosDt(ChVector3d(speed * c, speed * s, 0.0));
        const auto& shafts = car->GetSystem()->GetShafts();
        for (size_t i = 0; i < shafts.size() && i < shaft_ratio.size(); ++i) shafts[i]->SetPosDt(shaft_ratio[i] * spin);
        delta_ref = delta_act = 0.0;
        time = 0.0;
        x_start << X, Y, psi, speed, 0.0, 0.0, 0.0;
    }

    // Advance one step with Chrono's steering input in [-1, 1] and the total longitudinal force Fx [N].
    void advance(double steering, double Fx) {
        chrono::vehicle::DriverInputs in;
        in.m_steering = std::clamp(steering, -1.0, 1.0);
        in.m_throttle = 0.0;
        in.m_braking = 0.0;
        in.m_clutch = 0.0;
        const double axle_torque = 0.5 * std::max(Fx, 0.0) * tire_radius;  // per side of the driven axle
        for (auto side : {LEFT, RIGHT}) car->GetAxle(0)->m_suspension->ApplyAxleTorque(side, -axle_torque);
        car->Synchronize(time, in, ground);
        // the brake force split between the axles as the vehicle parameters say, not one input for all four
        const double brake_force = std::max(-Fx, 0.0);
        const double share[2] = {p.brake_front, 1.0 - p.brake_front};
        for (int axle = 0; axle < 2; ++axle) {
            const double torque = 0.5 * share[axle] * brake_force * tire_radius;  // per wheel
            const double braking = std::clamp(torque / BRAKE_TORQUE_MAX, 0.0, 1.0);
            car->GetAxle(axle)->m_brake_left->Synchronize(time, braking);
            car->GetAxle(axle)->m_brake_right->Synchronize(time, braking);
        }
        car->Advance(dt);
        time += dt;
    }

    // all brakes together, until the axle with the larger share reaches its peak torque
    double brake_force_max() const {
        return 2.0 * BRAKE_TORQUE_MAX / tire_radius / std::max(p.brake_front, 1.0 - p.brake_front);
    }

    // Mean steering angle of the two front wheels, from the direction of their spin axes in the vehicle frame.
    // Under load it differs a little from the commanded angle (suspension compliance, roll steer).
    double road_wheel_angle() const {
        double sum = 0.0;
        for (auto side : {LEFT, RIGHT}) {
            const ChVector3d axis = car->GetTransform().TransformDirectionParentToLocal(
                car->GetAxle(0)->m_suspension->GetSpindle(side)->GetRot().GetAxisY());
            sum += std::atan2(-axis.x(), axis.y());
        }
        return 0.5 * sum;
    }

    State observe() const {
        if (time == 0.0) return x_start;  // Chrono's own velocities are only current after the first step
        const ChVector3d pos = car->GetPointLocation(com), vel = car->GetPointVelocity(com);
        const ChVector3d forward = car->GetRot().GetAxisX();
        const double psi = std::atan2(forward.y(), forward.x());
        const double c = std::cos(psi), s = std::sin(psi);
        State x;
        // delta is what a steering-column sensor reports: the commanded road-wheel angle after the actuator
        x << pos.x(), pos.y(), psi, c * vel.x() + s * vel.y(), -s * vel.x() + c * vel.y(), car->GetYawRate(),
            delta_act;
        return x;
    }
};

ChronoPlant::ChronoPlant(const VehicleParams& params, double dt, double tau_steer) : impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    m.p = params;
    m.dt = dt;
    m.tau_steer = tau_steer;
    m.ground.mu = GROUND_MU;
    chrono::SetChronoDataPath(OV_CHRONO_DATA);
    chrono::vehicle::SetDataPath(OV_CHRONO_VEHICLE_DATA);

    // Measure the car at rest: where it settles, where its centre of mass is, how far the wheels steer.
    m.build(0.0, 0.0, 0.0, 0.0);
    m.tire_radius = m.car->GetAxle(0)->GetWheels()[0]->GetTire()->GetRadius();
    for (int i = 0; i < 2000; ++i) m.advance(0.0, -1e9);  // brakes on while it settles
    m.ride_height = m.car->GetTransform().GetPos().z();
    m.com = m.car->GetCOMFrame().GetPos();
    const double full_lock = m.car->GetMaxSteeringAngle();  // nominal, to scale the test input
    for (int i = 0; i < 500; ++i) m.advance(0.1, -1e9);  // a small angle, where the linkage is close to linear
    m.steer_gain = m.road_wheel_angle() / 0.1;
    if (!(m.steer_gain > 0.1 * full_lock)) m.steer_gain = full_lock;

    // How fast each driveline shaft turns when the car rolls freely.
    const double test_speed = 10.0;
    m.build(0.0, 0.0, 0.0, test_speed);
    for (int i = 0; i < 1500; ++i) m.advance(0.0, 0.0);
    const double spin = m.observe()[IUX] / m.tire_radius;
    for (const auto& shaft : m.car->GetSystem()->GetShafts()) m.shaft_ratio.push_back(shaft->GetPosDt() / spin);

    m.build(0.0, 0.0, 0.0, 0.0);
}

ChronoPlant::~ChronoPlant() = default;

void ChronoPlant::reset(const State& x0) {
    impl_->build(x0[IX], x0[IY], x0[IPSI], x0[IUX]);
    impl_->delta_ref = impl_->delta_act = impl_->x_start[IDELTA] = x0[IDELTA];
}

State ChronoPlant::observe() const { return impl_->observe(); }

// Actuator limits only: the tires limit the forces themselves.
Input ChronoPlant::saturate(const Input& u) const {
    const VehicleParams& p = impl_->p;
    const double Ux = std::max(impl_->observe()[IUX], p.Ux_min);
    const double fx_min = -std::min(p.F_brake_max, impl_->brake_force_max());
    const double fx_max = std::min(p.F_drive_max, p.P_max / Ux);
    return {std::clamp(u[IRATE], -p.delta_rate_max, p.delta_rate_max), std::clamp(u[IFX], fx_min, fx_max)};
}

State ChronoPlant::step(const Input& u, double duration) {
    Impl& m = *impl_;
    const double delta_max = std::min(m.p.delta_max, m.steer_gain);
    const int steps = static_cast<int>(std::lround(duration / m.dt));
    for (int i = 0; i < steps; ++i) {
        const Input v = saturate(u);
        m.delta_ref = std::clamp(m.delta_ref + v[IRATE] * m.dt, -delta_max, delta_max);
        m.delta_act = m.tau_steer > 0.0 ? m.delta_act + first_order_lag(m.delta_act, m.delta_ref, m.tau_steer) * m.dt
                                        : m.delta_ref;
        m.advance(m.delta_act / m.steer_gain, v[IFX]);
    }
    return observe();
}

void ChronoPlant::set_friction(double mu) { impl_->ground.mu = mu; }

ChronoPlant::Attitude ChronoPlant::attitude() const {
    const Impl& m = *impl_;
    if (m.time == 0.0) return {0.0, 0.0, 0.0};
    const chrono::ChQuaterniond q = m.car->GetRot();
    const ChVector3d forward = q.GetAxisX(), left = q.GetAxisY();
    const double pitch = -std::asin(std::clamp(forward.z(), -1.0, 1.0));  // nose down positive
    const double roll = std::asin(std::clamp(left.z(), -1.0, 1.0));  // left side up positive (leaning right)
    return {roll, pitch, m.car->GetPointLocation(m.com).z() - (m.ride_height + m.com.z())};
}

ChronoPlant::Facts ChronoPlant::facts() const {
    const Impl& m = *impl_;
    Sedan_Vehicle& car = *m.car;
    auto axle_x = [&](int axle) {
        return car.GetTransform()
            .TransformPointParentToLocal(car.GetAxle(axle)->m_suspension->GetSpindle(LEFT)->GetPos())
            .x();
    };
    return {car.GetMass(),
            car.GetInertia()(2, 2),
            axle_x(0) - m.com.x(),
            m.com.x() - axle_x(1),
            m.ride_height + m.com.z(),
            m.tire_radius,
            m.steer_gain,
            m.brake_force_max()};
}

}  // namespace sim
