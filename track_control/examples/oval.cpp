// The smallest complete use of the library: an oval, a car, both controllers.
//
// The car is simulated with the library's own bicycle model (no plant lags);
// in a real integration the simulator supplies the state and takes the input.
//
//   track_control_example_oval            pure pursuit and NMPC, one lap each
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"
#include "track_control/nmpc.hpp"
#include "track_control/pure_pursuit.hpp"

using namespace tc;

int main() {
    // a 250 x 90 m oval, driven anticlockwise, on the boundary between two 3.5 m lanes
    const Path path = Path::from_corners({{0, 0}, {250, 0}, {250, 90}, {0, 90}}, {35, 35, 35, 35}, {100, 0});
    Road road;
    road.v_max = 70.0 / 3.6;
    const VehicleParams params;  // CARLA's Tesla Model 3 by default
    const double right_lane = -0.5 * road.lane_width;

    PurePursuit::Options pp_options;
    pp_options.lane_offset = right_lane;
    pp_options.pass_offset = -right_lane;
    NMPC::Options nmpc_options;
    nmpc_options.lane_offset = right_lane;

    std::unique_ptr<Controller> controllers[] = {
        std::make_unique<PurePursuit>(path, road, params, Body(), pp_options),
        std::make_unique<NMPC>(path, road, params, Body(), nmpc_options),
    };

    for (auto& controller : controllers) {
        // start on the start line in the right lane at 30 km/h
        const Pose start = path.pose(0.0, right_lane);
        State x = State::Zero();
        x << start.x, start.y, start.psi, 30.0 / 3.6, 0.0, 0.0, 0.0;

        double t = 0.0, progress = 0.0, s_prev = 0.0, worst_offset = 0.0;
        while (progress < path.length() && t < 120.0) {
            Observation obs;
            obs.t = t;
            obs.x = x;
            Input u = controller->compute(obs);
            // what the car can do: steering rate and force limits
            const auto [fx_min, fx_max] = fx_limits(x[IUX], params);
            u[IRATE] = std::clamp(u[IRATE], -params.delta_rate_max, params.delta_rate_max);
            u[IFX] = std::clamp(u[IFX], fx_min, fx_max);
            for (int i = 0; i < 50; ++i) {  // hold the input for 50 ms, 1 ms steps
                x = rk4_step([&](const State& z, const Input& v) { return dynamics(z, v, params); }, x, u, 1e-3);
                x[IDELTA] = std::clamp(x[IDELTA], -params.delta_max, params.delta_max);
            }
            t += 0.05;
            const auto [s, e_y] = path.project(x[IX], x[IY]);
            progress += path.difference(s, s_prev);
            s_prev = s;
            worst_offset = std::max(worst_offset, std::fabs(e_y - right_lane));
        }
        std::printf("%-13s lap %5.1f s   furthest from the lane centre %.2f m\n", controller->name().c_str(), t,
                    worst_offset);
    }
    return 0;
}
