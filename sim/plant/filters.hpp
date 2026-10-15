// First-order lags the plant puts between a command and its effect.
#pragma once

namespace sim {

// Rate of change of `value` as it follows `target` with time constant `tau` [s].
inline double first_order_lag(double value, double target, double tau) { return (target - value) / tau; }

// Rate of change of `value` as it follows `target` over the distance `length` [m] at `speed` [m/s].
inline double relaxation(double value, double target, double speed, double length) {
    return speed / length * (target - value);
}

}  // namespace sim
