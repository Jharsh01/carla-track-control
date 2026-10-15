// NMPC on the dynamic bicycle model, in path coordinates (CasADi + IPOPT).
//
// The dynamics of [Ux, Uy, r, delta] and the inputs [delta_rate, Fx] are the
// bicycle model's (model/bicycle.hpp). The pose is carried relative to the
// path instead of in the world frame,
//
//     state  [s, e_y, e_psi, Ux, Uy, r, delta]
//     s'     = (Ux cos e_psi - Uy sin e_psi) / (1 - kappa e_y)
//     e_y'   = Ux sin e_psi + Uy cos e_psi
//     e_psi' = r - kappa s'
//
// so corners enter through the curvature kappa and the road edges are bounds
// on e_y. The curvature of each step of the horizon is a parameter, read from
// the path at the positions of the previous plan.
//
// Speed is not prescribed per corner: the cost asks for the speed limit, and
// the tire model with its friction limit, together with the road-edge and
// sideslip constraints, makes the plan brake for a corner it could not
// otherwise hold. Obstacles are kept out by a soft ellipse each, for the
// `n_cars` nearest ones, each predicted to hold its speed and lateral offset.
//
// If the road is known only `road_range` metres ahead, the plan continues
// beyond that with the last known curvature, and its speed there is limited to
// `unseen_speed`, so the car always keeps to a speed from which it can brake
// for whatever comes into view. The default is what the path's tightest corner
// allows at the road's friction limit, so that radius is taken as known.
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <vector>

#include "track_control/controller.hpp"

namespace tc {

class NMPC : public Controller {
public:
    static constexpr int NS = 4;  // slacks per step: obstacle ellipses, road edges, sideslip, speed limit

    // `lane_offset` is the line kept when nothing is in the way. The ellipse
    // keeps the bodies `margin_lat` apart side by side and `margin_long` apart
    // nose to tail (obstacles are taken to have the ego's size); `edge_margin`
    // is kept between the body and the road edge.
    struct Options {
        double horizon = 4.0;  // [s]
        double h = 0.1;  // step [s]
        double lane_offset = 0.0;  // [m]
        double road_range = INF;  // [m]
        std::optional<double> unseen_speed;  // [m/s]
        int n_cars = 2;
        double margin_lat = 0.6;  // [m]
        double margin_long = 1.0;  // [m]
        double edge_margin = 0.2;  // [m]
        double sideslip_max = 7.0 * M_PI / 180.0;  // [rad]
        double w_speed = 1.0;
        double w_lane = 0.2;
        double w_lat = 1.0;
        double w_accel = 1.0;
        double w_rate = 100.0;
        double w_slack = 1e3;
        int max_iter = 200;  // IPOPT iterations per solve
    };

    NMPC(Path path, Road road, VehicleParams params, Body body, Options options);
    NMPC(Path path, Road road, VehicleParams params) : NMPC(std::move(path), road, params, Body(), Options()) {}
    ~NMPC() override;

    Input compute(const Observation& obs) override;
    std::string name() const override { return "nmpc"; }

    // World state as the model's [s, e_y, e_psi, Ux, Uy, r, delta].
    State model_state(const State& x) const;

    const Path path;
    const Road road;
    const VehicleParams p;
    const Body body;
    const Options opt;
    const double h;
    const int N;
    const double unseen_speed;

    std::vector<double> solve_times;  // wall time of each solve [s]
    int failures = 0;  // solves that did not converge; their result is still applied
    Eigen::MatrixXd plan;  // last predicted states in path coordinates, (N + 1) x NX; empty before the first solve

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace tc
