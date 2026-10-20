# track_control

Pure pursuit and nonlinear MPC for following a path at the limits of grip, in C++17.
Simulator-independent: you give the controller the car's state and the cars around it,
it gives back a steering rate and a longitudinal tire force.

- **`tc::PurePursuit`**: pure-pursuit steering on a line offset from the path, a
  grip-limited speed profile (friction circle, brake and drive limits), and a lane-change
  rule for passing slower cars.
- **`tc::NMPC`**: a receding-horizon optimal control problem on a dynamic single-track
  model with Fiala tires and load transfer, in path coordinates (CasADi + FATROP, or IPOPT). The tire
  model's friction limit sets the cornering speed; road edges, sideslip and other cars are
  soft constraints.
- **`tc::Path`**: the reference line as straights and arcs, built from corner radii, from
  waypoints (biarc fit: passes through every point, continuous heading) or from pieces;
  closed (a lap) or open.

Dependencies: Eigen 3.3+, CasADi 3.6+ with FATROP and IPOPT (the pip wheel works, it ships a CMake
package), nlohmann/json (fetched if not installed).

## Steering the NMPC from a supervisor (e.g. a VLA model)

Every cost weight and limit of `tc::NMPC` is a parameter of its problem, so a supervisor can
change them between one solve and the next without rebuilding the solver (`directive.hpp`):

- **weights** by cost term: `speed`, `lane`, `lat`, `accel`, `rate`, `slack`
- **limits**: `speed_cap`, `lane_offset`, `margin_lat`, `margin_long`, `edge_margin`, `sideslip_max`
- **zones** to keep out of, in path coordinates (a pedestrian, debris, a closed lane), at most
  `Options::n_zones` at a time, the nearest kept. The default 0 leaves the problem exactly as it is
  without a supervisor; set it (e.g. 4) when one is connected
- **intents** mapped to the above by `NMPC::interpret`: `keep_lane`, `lane_left`, `lane_right`,
  `set_speed`, `more_gap`, `cautious`, `avoid`

```cpp
nmpc.set_directive(nmpc.interpret({{tc::Intent::set_speed, 10.0}, {tc::Intent::lane_left}}));
```

Values are clamped to `weight_ranges()`/`limit_ranges()`. The ranges only let a directive make the
car more careful than its defaults or tune its performance within limits. The speed cap can't
exceed the road's limit, margins and the slack weight can't go below their defaults, and the
sideslip limit can't go above its default. Road edges, friction and actuator limits are fixed.
`set_directive` returns what it clamped and throws on a name it doesn't know. A directive lapses
`ttl` seconds (default 2) after it takes effect, so a supervisor that stops sending leaves the car
on its defaults. It may be called from any thread.

To add a new kind of cost term or constraint, add one entry to `cost_terms()` or
`constraint_groups()` in `src/nmpc.cpp`. A new cost term automatically gets a weight a directive
can set. In Python (`trackctl`): `driver.set_directive(weights=..., limits=..., zones=[trackctl.Zone(...)])`,
`driver.set_intents([trackctl.Intent("set_speed", 10.0)])`, `driver.directive_ranges`, `driver.clear_directive()`.

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
