// Both controllers in closed loop on the library's bicycle model: they follow the lane, hold speed, and pass.
#include <algorithm>
#include <cmath>

#include "check.hpp"
#include "track_control/model/bicycle.hpp"
#include "track_control/model/integrate.hpp"
#include "track_control/nmpc.hpp"
#include "track_control/pure_pursuit.hpp"

using namespace tc;

namespace {

const Path PATH = Path::from_corners({{0, 0}, {200, 0}, {200, 80}, {0, 80}}, {30, 30, 30, 30}, {60, 0});
const double RIGHT = -1.75;

struct Run {
    double worst_offset = 0.0;  // furthest from the right lane centre [m]
    double min_gap = INFINITY;  // closest approach to the obstacle, along the path [m]
    double end_speed = 0.0;
    double distance = 0.0;
};

// Drive for `duration` seconds; an obstacle, if `obstacle_speed` is set, starts 40 m ahead in the right lane.
Run drive(Controller& controller, double duration, double obstacle_speed = -1.0) {
    const VehicleParams p;
    const Pose start = PATH.pose(0.0, RIGHT);
    State x = State::Zero();
    x << start.x, start.y, start.psi, 12.0, 0.0, 0.0, 0.0;
    Run run;
    double s_prev = 0.0;
    for (double t = 0.0; t < duration; t += 0.05) {
        Observation obs;
        obs.t = t;
        obs.x = x;
        const double obstacle_s = 40.0 + obstacle_speed * t;
        if (obstacle_speed >= 0.0) obs.obstacles.push_back({PATH.wrap(obstacle_s), RIGHT, obstacle_speed});
        Input u = controller.compute(obs);
        const auto [fx_min, fx_max] = fx_limits(x[IUX], p);
        u[IRATE] = std::clamp(u[IRATE], -p.delta_rate_max, p.delta_rate_max);
        u[IFX] = std::clamp(u[IFX], fx_min, fx_max);
        for (int i = 0; i < 50; ++i)
            x = rk4_step([&](const State& z, const Input& v) { return dynamics(z, v, p); }, x, u, 1e-3);
        const auto [s, e_y] = PATH.project(x[IX], x[IY]);
        run.distance += PATH.difference(s, s_prev);
        s_prev = s;
        if (obstacle_speed >= 0.0) {
            const double gap = std::fabs(PATH.difference(PATH.wrap(obstacle_s), s));
            if (std::fabs(e_y - RIGHT) < 2.0) run.min_gap = std::min(run.min_gap, gap);  // only while in its lane
        } else {
            run.worst_offset = std::max(run.worst_offset, std::fabs(e_y - RIGHT));
        }
    }
    run.end_speed = x[IUX];
    return run;
}

Road road() {
    Road r;
    r.v_max = 15.0;
    return r;
}

}  // namespace

static void pure_pursuit_follows() {
    PurePursuit::Options o;
    o.lane_offset = RIGHT;
    o.pass_offset = -RIGHT;
    PurePursuit controller(PATH, road(), VehicleParams(), Body(), o);
    const Run run = drive(controller, 55.0);  // more than a lap
    CHECK(run.distance > PATH.length());
    CHECK(run.worst_offset < 0.8);
}

static void pure_pursuit_passes() {
    PurePursuit::Options o;
    o.lane_offset = RIGHT;
    o.pass_offset = -RIGHT;
    PurePursuit controller(PATH, road(), VehicleParams(), Body(), o);
    const Run run = drive(controller, 25.0, 5.0);
    CHECK(run.distance > 40.0 + 5.0 * 25.0 + 10.0);  // got past it
    CHECK(run.min_gap > 5.0);  // never close behind it in its lane
}

static void nmpc_follows_and_passes() {
    NMPC::Options o;
    o.lane_offset = RIGHT;
    o.horizon = 3.0;
    {
        NMPC controller(PATH, road(), VehicleParams(), Body(), o);
        const Run run = drive(controller, 20.0);
        CHECK(run.distance > 250.0);
        CHECK(run.worst_offset < 0.8);
        CHECK_NEAR(run.end_speed, 15.0, 1.5);
        CHECK(controller.failures == 0);
    }
    {
        NMPC controller(PATH, road(), VehicleParams(), Body(), o);
        const Run run = drive(controller, 20.0, 5.0);
        CHECK(run.distance > 40.0 + 5.0 * 20.0 + 10.0);
        CHECK(run.min_gap > 5.0);
    }
}

int main() {
    pure_pursuit_follows();
    pure_pursuit_passes();
    nmpc_follows_and_passes();
    return check_report("test_controllers");
}
