"""The controllers' input [steering rate, Fx] as CARLA's throttle, brake and steer.

Steering: the rate is integrated into a road-wheel angle (within the vehicle's
limits), which CARLA takes as a fraction of the front wheels' maximum steering
angle. The bridge flattens the vehicle's speed-dependent steering curve
(world.py), so the fraction means the same angle at every speed. CARLA's steer
is positive to the right; the library's angle is positive to the left.

Longitudinal: Fx is the total tire force the controller wants. Throttle is
taken as proportional to drive force up to what the drive can deliver at the
current speed (min(F_drive_max, P_max / Ux)), brake as proportional to brake
force up to F_brake_max, as the vehicle parameters for the Model 3 were derived
(docs/vehicle_parameters.md). CARLA's engine, gearbox and tires do not follow
that exactly, so a PI loop on the longitudinal acceleration adds the force
needed to make the car accelerate as the model expects:

    expected  a = (Fx - drag(Ux)) / m        measured  a = dUx/dt
"""
import math


class Actuation:
    def __init__(self, params, max_steer_deg, dt, kp=0.3, ki=0.6, i_max=2.0):
        """`params`: trackctl.VehicleParams; `max_steer_deg`: CARLA's front-wheel max steer angle; `dt`: control period."""
        self.p = params
        self.max_steer = math.radians(max_steer_deg)
        self.dt = dt
        self.kp, self.ki, self.i_max = kp, ki, i_max  # gains on the acceleration error [-, 1/s], integral cap [m/s^2]
        self.delta = 0.0  # commanded road-wheel angle [rad]
        self.integral = 0.0
        self.prev_Ux = None
        self.prev_expected = None

    def reset(self, delta=0.0):
        self.delta = delta
        self.integral = 0.0
        self.prev_Ux = None
        self.prev_expected = None

    def drag(self, Ux):
        p = self.p
        return 0.5 * p.rho * p.CdA * Ux * Ux + p.Crr * p.m * p.g

    def command(self, u, Ux):
        """(throttle, brake, steer) for the input u = [rate, Fx] held over the next period, at speed Ux."""
        p = self.p
        rate = max(-p.delta_rate_max, min(p.delta_rate_max, u[0]))
        limit = min(p.delta_max, self.max_steer)
        self.delta = max(-limit, min(limit, self.delta + rate * self.dt))
        steer = max(-1.0, min(1.0, -self.delta / self.max_steer))

        # acceleration error over the last period, against what the last command should have given
        if self.prev_Ux is not None:
            measured = (Ux - self.prev_Ux) / self.dt
            error = self.prev_expected - measured
            self.integral = max(-self.i_max, min(self.i_max, self.integral + error * self.dt))
        else:
            error = 0.0
        self.prev_Ux = Ux
        self.prev_expected = (u[1] - self.drag(Ux)) / p.m

        F = u[1] + p.m * (self.kp * error + self.ki * self.integral)
        if F >= 0.0:
            available = min(p.F_drive_max, p.P_max / max(Ux, 1.0))
            return min(1.0, F / available), 0.0, steer
        return 0.0, min(1.0, -F / p.F_brake_max), steer
