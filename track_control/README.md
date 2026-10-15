# track_control

Pure pursuit and nonlinear MPC for following a path at the limits of grip, in C++17.
Simulator-independent: you give the controller the car's state and the cars around it,
it gives back a steering rate and a longitudinal tire force.

- **`tc::PurePursuit`**: pure-pursuit steering on a line offset from the path, a
  grip-limited speed profile (friction circle, brake and drive limits), and a lane-change
  rule for passing slower cars.
- **`tc::NMPC`**: a receding-horizon optimal control problem on a dynamic single-track
  model with Fiala tires and load transfer, in path coordinates (CasADi + IPOPT). The tire
  model's friction limit sets the cornering speed; road edges, sideslip and other cars are
  soft constraints.
- **`tc::Path`**: the reference line as straights and arcs, built from corner radii, from
  waypoints (biarc fit: passes through every point, continuous heading) or from pieces;
  closed (a lap) or open.

Dependencies: Eigen 3.3+, CasADi 3.6+ with IPOPT (the pip wheel works, it ships a CMake
package), nlohmann/json (fetched if not installed).

## Build, test, install

```bash
CASADI_CMAKE=$(python -c "import casadi, os; print(os.path.join(os.path.dirname(casadi.__file__), 'cmake'))")
cmake -S track_control -B build -Dcasadi_DIR=$CASADI_CMAKE
cmake --build build -j
ctest --test-dir build                     # path, model and closed-loop controller tests
build/track_control_example_oval           # both controllers round an oval
cmake --install build --prefix ~/.local    # then find_package(track_control) works
```

With CasADi built from source (default C++ string ABI), add `-DTRACK_CONTROL_OLD_CXX_ABI=OFF`;
the pip wheels use the old ABI, and so must everything that passes strings to them.

## Use

```cmake
find_package(track_control CONFIG REQUIRED)        # or add_subdirectory(track_control)
target_link_libraries(my_app PRIVATE track_control::track_control)
```

```cpp
#include <track_control/nmpc.hpp>
#include <track_control/pure_pursuit.hpp>

// the route, e.g. CARLA waypoints converted to a right-handed frame (y = -carla_y)
tc::Path path = tc::Path::from_waypoints(points, /*closed=*/false);
tc::Road road;                     // lane width, distance to the road edge, friction, speed limit
road.v_max = 50.0 / 3.6;
tc::VehicleParams car = tc::VehicleParams::from_file("model3.json");   // or set the fields

tc::NMPC::Options options;
options.lane_offset = -1.75;       // keep to the right lane of a road whose path is the centre line
tc::NMPC nmpc(path, road, car, tc::Body(), options);

// once per control period (the NMPC is set up for 50 ms):
tc::Observation obs;
obs.t = t;
obs.x << X, Y, psi, Ux, Uy, r, delta;              // centre of gravity, right-handed, SI units
obs.obstacles.push_back({s, e_y, speed});          // other cars, in path coordinates
tc::Input u = nmpc.compute(obs);                   // [steering rate rad/s, total tire force N]
```

Conventions (`types.hpp`): right-handed world frame, heading counter-clockwise, the state at
the centre of gravity, the steering angle positive to the left, Fx positive to drive. To drive
a simulator that takes pedal and steering-wheel commands, integrate the steering rate into an
angle and map Fx to throttle and brake; `carla_bridge/actuation.py` in the repository this
library comes from does that for CARLA, with a small acceleration loop for the engine map.

## Choosing parameters

The NMPC plans with the model, so its parameters decide how close to the limit it drives. The
repository it ships in identifies them for two cars and shows what goes wrong when they are
off: for the Project Chrono sedan, a single friction coefficient for both axles made the model
understeer where the car is near-neutral, and the NMPC spun it; fitting each axle's tire curve
(`grip_f`, `grip_r`) fixed that (see that repository's README).

## License

MIT
