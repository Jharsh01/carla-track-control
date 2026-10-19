# Real-time plant, threaded CARLA bridge and NMPC fallback: what changed

This report covers one set of changes on top of `f18a533`. They come in six parts:

1. The bicycle plant runs in real time at 1 kHz on its own thread, and its state is a global that
   any controller can read.
2. Cornering stiffness depends on load, from a lookup table in the vehicle file.
3. In the CARLA bridge, the plant, the controller, the CARLA world and the drawing each run on
   their own thread.
4. The NMPC runs with pure pursuit as a fallback, and a command goes out every 10 ms whichever
   controller produced it.
5. The live view is faster: the CARLA world thread went from 13.6 Hz to 24.4 Hz.
6. A build fix removes an intermittent crash caused by Chrono's compiler flags.

The final section lists the measurements, what was tested and the open issues.

---

## 1. Bicycle plant on a 1 kHz real-time thread, with global state

**Files:** `sim/plant/bicycle_plant.{hpp,cpp}`, `sim/plant/test_bicycle_realtime.cpp`

**Global state.** `sim::bicycle_state` is a `BicycleBus` that holds the plant's state, the input
and the friction:

```cpp
double t;
State x = bicycle_state.state(&t);   // a consistent [X, Y, psi, Ux, Uy, r, delta] and its plant time
double ux = bicycle_state[IUX];      // one state variable
bicycle_state.command(u);            // the input the plant holds from its next tick
bicycle_state.mu = 0.9;              // friction under the car
bicycle_state.applied_input();       // the input after the plant's limits
```

Each signal is protected by a sequence lock. The plant writes without ever waiting, and a reader
that overlaps a write simply reads again, so no call ever blocks the plant thread.

**The thread.** `BicyclePlant::start({rate, priority, cpu})`, `stop()` and `timing()` control it.

- **Timer:** a kernel timer (`timerfd`, `CLOCK_MONOTONIC`) wakes the thread at fixed points in time
  (start + k × 1 ms), so the rate never drifts.
- **Late ticks:** if the thread wakes late, the timer reports how many periods were missed and the
  plant steps through all of them. Plant time therefore always equals wall-clock time, and each
  missed tick is counted as an overrun.
- **Priority:** the thread asks for `SCHED_FIFO` and falls back to normal scheduling if that isn't
  allowed. `timing()` reports which one it got, the number of ticks and overruns, and the wake-up
  latency.
- **Friction:** if `surface_friction` is set, the plant looks up the friction under the car on
  every tick.
- **Offline use:** `step()` still works whenever the thread isn't running, and throws while it is.

**Measured** over 0.5 s at normal priority: 501 ticks, 0 overruns, wake-up latency 19 µs mean and
95 µs max. The final state matched the offline plant to within 1e-9.

## 2. Cornering stiffness against axle load, as a lookup table

**Files:** `config/vehicles/model3.json`, `track_control/include/track_control/{vehicle_params.hpp, model/tire.hpp, model/bicycle.hpp}`, `track_control/src/vehicle_params.cpp`, `docs/vehicle_parameters.md`

- **The table.** `C_alpha_f` and `C_alpha_r` in the vehicle file now carry
  `"load_table": {"Fz": [...], "C_alpha": [...]}`, read into `VehicleParams::C_alpha_{f,r}_load`.
  For the Model 3 the table has 10 points from 0.25× to 3× the axle's static load. It follows the
  PhysX per-tire curve already assumed in `docs/vehicle_parameters.md`, so the values at static
  load are unchanged: 106.5 and 67.8 kN/rad.
- **The model.** The bicycle model looks up each axle's stiffness at its current load, which
  shifts under braking and drive. Between table points it uses a smooth, monotone curve
  (`cornering_stiffness`). Straight-line interpolation was tried first and made the NMPC fail to
  converge 20 times in one lap.
- **Understeer gradient.** `understeer_gradient(Fx)` now accounts for the longitudinal force. For
  the Model 3 it is −1.6°/g at full brake, 0 when coasting and +2.8°/g at full drive.
- **Who uses the table.** The plant does. The NMPC plans with the static-load values, because
  planning with the table doubled its solve time (105 → 209 ms) and changed nothing in the lap.
- **New tests** in `track_control/tests/test_model.cpp`: interpolation, reading the table from a
  file, and two runs from a steady corner. At full brake the rear saturates first (at 0.5 s, the
  front at 1.0 s) and the car spins. At full drive the front saturates first and the car runs wide.

## 3. CARLA bridge: plant, controller, world and drawing on their own threads

**Files:** `carla_bridge/{run.py, world.py, view.py}`, `bindings/trackctl.cpp`

With `--dynamics bicycle` the bridge no longer runs one loop. The pieces run side by side in real
time and share the state and input through `trackctl.bicycle_state`:

| Thread | Rate | Work |
|---|---|---|
| Plant (C++) | `--plant-rate` (1000 Hz) | steps the bicycle model; looks up the friction under the car |
| Controller | `--control-rate` (100 Hz) | reads the state, solves, commands the input; the solve releases Python's GIL |
| CARLA world (main) | `--world-rate` (25 Hz) | moves the car and traffic, ticks CARLA, reads the camera, logs, ends the lap |
| View | as fast as it can, up to the world rate | draws the HUD and rate panel, writes the video and the live stream |

- **Lockstep mode:** `--lockstep` brings back the old single loop. CARLA and Chrono dynamics
  always run in lockstep.
- **Python bindings:** `BicyclePlant.start/stop/timing/running`, `trackctl.bicycle_state` (with
  `state()`, `command()`, `applied`, `mu`) and `Tuning.solve_time_limit` are new. Calling a
  `Driver` now releases the GIL.
- **Rate panel:** it plots each thread's actual frequency: three plots in threaded mode, five with
  the NMPC fallback.

## 4. NMPC with pure pursuit as its fallback

**Files:** `sim/racetrack/fallback_control.{hpp,cpp}` (new), `track_control/{include/track_control/nmpc.hpp, src/nmpc.cpp}`, `sim/racetrack/driver.{hpp,cpp}`

Run it with `./run.sh --controller nmpc_fallback --dynamics bicycle [--nmpc-deadline 0.1] [--control-rate 100]`.

Three C++ threads do the work, so Python's GIL can't disturb the 100 Hz timing:

- **NMPC thread.** It solves back to back. Each solve has `--nmpc-deadline` (100 ms) of wall time,
  enforced through IPOPT's `max_wall_time`. Each solve plans from where the car will be when the
  deadline is up, predicted with the bicycle model while holding the input currently applied.
  A solve that makes the deadline therefore delivers its plan before the plan starts.
- **Pure pursuit thread.** It computes a backup command every 10 ms.
- **Command thread.** Every 10 ms it sends the NMPC's planned input for that moment, as long as the
  NMPC is on time. Otherwise it sends pure pursuit's latest input. The NMPC counts as late if its
  solve didn't converge, took longer than the deadline, or hasn't finished by the time its plan
  should start.

The NMPC library gained `Options::max_wall_time`, `plan_inputs` (the planned input sequence) and
`last_success`.

**Which controller is in command** is shown on screen and saved in the logs:

- a HUD banner, green "COMMAND: NMPC" with the plan's age and solve time, or orange
  "COMMAND: PURE PURSUIT (fallback)" with the reason;
- a `command` column in the `.csv`;
- `nmpc_command_share`, `nmpc_solves` and `nmpc_solves_on_time` in the `.json`.

## 5. Live view performance

On real CARLA the world thread ran at 13.6 Hz. Profiling with py-spy showed that its CARLA work,
the tick and the camera read, took about 29 ms per frame. Another 44 ms went on drawing and
encoding, all in the same thread. The changes:

- **Separate view thread:** drawing moved to its own thread. When it falls behind it skips a frame
  and the video repeats the previous one, so the video keeps real-time pace.
- **Rate panel:** redrawn 5 times a second and reused in between, drawn from a locked snapshot of
  its samples.
- **HUD map:** the map box and track outline are drawn once and pasted each frame (HUD 17.7 →
  12.7 ms).
- **Camera image:** converted from BGRA to RGB by PIL (2 ms) instead of a numpy copy (8 ms).
- **Video:** x264 uses `-preset veryfast`. ffmpeg had been using 2.3 cores at the default preset.
- **Live stream:** no JPEGs are encoded unless a browser has the stream open.

**Result on real CARLA (Epic quality, NVIDIA L4) with a stream viewer:** the world thread runs at
**24.4 Hz** and the view thread at 24.4 Hz, drawing 367 of 367 frames.

## 6. Build fix: intermittent crash from Chrono's compiler flags

**File:** `CMakeLists.txt`

Chrono's CMake package replaces `CMAKE_CXX_FLAGS` with the flags Chrono was built with,
`-O3 -fopenmp -march=native`, and on this CPU that includes AVX-512. The project looked for Chrono
after adding `track_control`, so the two halves of the build disagreed:

- `track_control` assumed 16-byte-aligned Eigen memory;
- `sim` and `trackctl` assumed 64-byte alignment.

Copying an Eigen matrix allocated in `track_control` (the NMPC's planned inputs) inside `sim`
segfaulted on an AVX-512 aligned load (`vmovapd zmm`) whenever the allocation wasn't 64-byte
aligned. The project now looks for Chrono first, so everything compiles with the same flags. Builds
without Chrono, such as CI, are unaffected.

---

## Measurements and findings

| | Result |
|---|---|
| Offline laps after all changes | NMPC 72.3 s, pure pursuit 98.5 s (unchanged) |
| Real-time NMPC alone, threaded, 100 ms solves | left the track after 6.8 s: inputs ~100 ms stale, iterations 13 → 31 |
| Pure pursuit, threaded, full lap (stand-in CARLA) | 90.5 s; plant 1000.0 Hz, controller and world at their set rates |
| NMPC fallback, 100 ms deadline, real CARLA | 98.3 s lap, clean; ~1 of 140 solves on time, so pure pursuit drove |
| NMPC fallback, 200 ms deadline (stand-in) | NMPC sent about half the commands; left the track in 2 of 3 runs |

- **The NMPC at 100 ms.** With every thread competing for the CPU, IPOPT gets about 9 of the ~13
  iterations a solve needs in 100 ms.
- **Spins at 200 ms.** The two controllers want different lines and speeds. In the traced run,
  pure pursuit took over in the middle of a turn, braked fully and steered the other way: the
  brake-in-turn oversteer described in section 2.
- **Plant overruns on real CARLA.** At normal priority on the loaded VM about 5 % of ticks are
  late. All are caught up, so the plant still averages 1000 Hz.

## Tests

- **ctest (4 tests):** includes the new `sim.test_bicycle_realtime`, and `test_model` with the
  load-table and oversteer/understeer checks.
- **pytest (17 tests, including the slow full laps):** the new tests cover the plant thread and
  `bicycle_state` from Python, the three threads running side by side, the NMPC fallback (100 Hz
  commands, the `command` column, the NMPC's share), the command banner, and the threaded rate
  panel.

## Open issues

1. **Make the NMPC fit 100 ms:** shorten the horizon from 4 s to about 2 s so most solves make the
   deadline and handovers become rare.
2. **Smoother handovers:** pure pursuit could follow the NMPC's last planned path, or steering rate
   and brake force could be limited at a handover.
3. **Real-time priority:** for the plant, `sudo setcap cap_sys_nice+ep` on the Python binary, or a
   higher `ulimit -r`.
4. **Single NMPC in threaded mode:** it has no latency compensation; only the fallback mode has.
