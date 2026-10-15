// State and input of the vehicle, as every part of the library uses them.
//
// State  x = [X, Y, psi, Ux, Uy, r, delta]
// Input  u = [delta_rate, Fx]
//
// X, Y [m] are the position of the centre of gravity and psi [rad] the heading,
// in a right-handed world frame (x forward/east, y left/north, psi
// counter-clockwise). Ux, Uy [m/s] are the body-frame velocities at the centre
// of gravity, r [rad/s] the yaw rate and delta [rad] the road-wheel steering
// angle. The input is the steering rate [rad/s] and the total longitudinal tire
// force Fx [N], positive to drive and negative to brake.
//
// A simulator with another convention (CARLA is left-handed) converts at its
// boundary; see carla_bridge/frames.py for CARLA.
#pragma once

#include <Eigen/Dense>

namespace tc {

constexpr int NX = 7;
constexpr int NU = 2;
enum StateIndex { IX, IY, IPSI, IUX, IUY, IR, IDELTA };
enum InputIndex { IRATE, IFX };

using State = Eigen::Matrix<double, NX, 1>;
using Input = Eigen::Matrix<double, NU, 1>;

}  // namespace tc
