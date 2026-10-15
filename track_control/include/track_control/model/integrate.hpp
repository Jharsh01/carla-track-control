// Fixed-step integration.
#pragma once

namespace tc {

// One classical Runge-Kutta step of dx/dt = f(x, u) with u held constant.
template <class F, class X, class U>
X rk4_step(F&& f, const X& x, const U& u, double dt) {
    X k1 = f(x, u);
    X k2 = f(X(x + 0.5 * dt * k1), u);
    X k3 = f(X(x + 0.5 * dt * k2), u);
    X k4 = f(X(x + dt * k3), u);
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace tc
