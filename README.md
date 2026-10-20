# carla-track-control

A nonlinear MPC and a pure-pursuit baseline driving a race track in CARLA, on either CARLA's own
vehicle physics or a Project Chrono multibody car, with slower traffic to pass. The controllers
are a standalone C++ library, [`track_control/`](track_control/), meant to be dropped into other
CARLA (or non-CARLA) projects.

```
clone ─► ./setup.sh ─► ./run.sh ─► results/carla/nmpc_carla.mp4, pure_pursuit_carla.mp4 (+ .csv, .json)
```

## The scenario

A 1.29 km circuit drawn from a hand sketch: two 3.5 m lanes of asphalt with a 1.75 m grass strip
each side and a barrier beyond, eight corners from 15 m to 40 m radius, driven clockwise
(`config/racetrack.json`). The ego car starts on the start line at 50 km/h in the right lane with
an 80 km/h limit; six slower cars (one at 30 km/h, five at 50 km/h) hold the right lane. Sensing is
imperfect by default: traffic seen 60 m ahead and 20 m behind with noise, a noisy ego state, and
the road's shape known only 40 m ahead (`--perfect` turns that off).

| | NMPC | Pure pursuit |
|---|---|---|
| Steering | optimised over a 4 s horizon on a dynamic bicycle model | pure pursuit on the lane line |
| Speed | whatever the tire model allows, up to the limit | grip-limited speed profile, 40 % of μg |
| Passing | soft ellipse around the 2 nearest cars | rule: change lane when a car is in the way |

In CARLA the track is generated from OpenDRIVE at start-up (`client.generate_opendrive_world`),
so no extra map download is needed and the road CARLA draws is the path the controllers follow.

CARLA draws that generated road and its walls in one flat untextured material over an empty void,
so the bridge hides them (they still collide) and draws the circuit itself
(`carla_bridge/scenery.py`):

- dark asphalt with white edge lines
- red and white kerbs on every corner
- a chequered start line
- mown grass to the horizon
- Armco along the walls
- woods, and a grandstand with a crowd and sponsor boards on the start straight

All of it is CARLA's own meshes and textured planes, spawned when the world loads. That adds about
3,400 actors and about 8 s at start-up; on an L4 a world tick then takes about 17 ms, well under
the NMPC's solve time. `--no-scenery` brings back the
plain road.

## Quick start on NVIDIA Brev

1. Create a GPU instance in **VM mode** (CARLA refuses to run as root, and VM mode gives you the
   `ubuntu` user), Ubuntu 22.04, a GPU with 16 GB or more (L4, A10G), about 60 GB of disk.
   Brev's image comes with the NVIDIA driver; on a plain cloud image install it first
   (`sudo apt-get install -y nvidia-driver-580-server nvidia-utils-580-server && sudo modprobe nvidia`).
   `./setup.sh` stops at the start if `nvidia-smi` or Vulkan cannot see the GPU.
2. On the instance:

   ```bash
   cd ~/workspace                                     # persists across restarts on Brev
   git clone https://github.com/<you>/carla-track-control.git && cd carla-track-control
   ./setup.sh                                         # ~1 h: CARLA download (8.4 GB), Chrono build
   ./run.sh                                           # NMPC then pure pursuit, CARLA dynamics
   ./run.sh --dynamics chrono                         # the same on the Project Chrono sedan
   ```

3. Get the videos: `scp '<instance>:~/workspace/carla-track-control/results/carla/*.mp4' .`
   (`brev ls` shows the instance name; `brev refresh` puts it in your SSH config).

### Watching live

The server renders off screen, so the bridge serves its own camera view:

```bash
ssh -L 8080:localhost:8080 <instance>      # on your laptop
./run.sh --controller nmpc --stream 8080   # on the instance; then open http://localhost:8080
```

### Options

`./run.sh -h` and `python -m carla_bridge -h` list them all. The main ones:

| | |
|---|---|
| `--controller nmpc\|pure_pursuit\|both` | which controller(s) to run, one after the other (default both) |
| `--dynamics carla\|chrono\|bicycle` | who simulates the ego car (default carla) |
| `--perfect` | exact sensing, the whole road known |
| `--road-range M`, `--seed N` | how far ahead the road is known; the noise seed |
| `--laps N`, `--t-max S` | |
| `--stream PORT`, `--no-video` | live view; no camera at all (fastest) |
| `--no-rates` | no side panel plotting the controller's and the plant's rate in wall-clock Hz (and the CARLA world's, threaded) |
| `--no-inputs` | no side panel plotting the controller's inputs, acceleration and steering rate (with `nmpc_fallback`, the NMPC's and pure pursuit's both, and whose the plant got) |
| `--controller nmpc_fallback` | with `--dynamics bicycle`: the NMPC and pure pursuit both running, the input blended to pure pursuit's whenever the NMPC has no plan for the moment (below) |
| `--control-rate HZ` | threaded: how often an input is sent (default 50 Hz with `nmpc_fallback`, otherwise 100 Hz) |
| `--nmpc-period S`, `--blend S` | `nmpc_fallback`: an NMPC solve every S seconds, each plan played for one period (default 0.2 s: 10 commands at 50 Hz); how long a handover between the two controllers takes (default 0.2 s, 0 for a switch) |
| `--nmpc-dt S`, `--nmpc-horizon S` | the NMPC's step and how far it plans (defaults 0.1 s and 4 s; 2 s with `nmpc_fallback`; `build/simulate` takes the same) |
| `--nmpc-fine-dt S`, `--nmpc-fine S` | a finer NMPC step for the start of the horizon (default with `nmpc_fallback`: one command, 20 ms, for one period, 0.2 s; otherwise none) |
| `--lockstep` | with `--dynamics bicycle`: one loop (controller, plant, CARLA tick) instead of three threads |
| `--plant-rate HZ`, `--world-rate HZ` | the threaded bicycle plant's rate (default 1000); the CARLA world thread's: ticks, camera frames, video (default 25) |
| `--no-scenery` | CARLA's plain generated road instead of the drawn circuit |
| `--docker` | start CARLA from the `carlasim/carla:0.9.15` image instead of `external/carla` |
| `--offline` | no CARLA: the C++ simulation (`build/simulate -h`) |
| `--stop-carla` | stop the server (it is left running between runs) |

## The two vehicle dynamics

**`--dynamics carla`**: CARLA's PhysX vehicle (`vehicle.tesla.model3`). The controllers plan with
`config/vehicles/model3.json`, parameters read from CARLA's physics control or derived from it,
some assumed (`docs/vehicle_parameters.md`). The bridge integrates the steering rate into an angle
and sends it as a fraction of the wheels' maximum (with CARLA's speed-dependent steering curve
flattened), and turns the force into throttle or brake, with a PI loop on the acceleration that
absorbs the engine map (`carla_bridge/actuation.py`).

**`--dynamics chrono`**: the Project Chrono sedan (multibody chassis and suspension, Pacejka
PAC2002 tires, driveline), stepped inside the bridge at 1 ms; CARLA only draws the car, with
Chrono's roll and pitch. The controllers plan with `config/vehicles/chrono_sedan.json`, identified
from the Chrono model. The grass's lower friction applies here only: CARLA's surface is uniform.
How the plant works, why each step needs a solver and where its time goes:
[`docs/chrono_plant.pdf`](docs/chrono_plant.pdf).

**`--dynamics bicycle`**: the offline plant (the controllers' own dynamic bicycle model with the
actuator lags, `config/vehicles/model3.json`), stepped in real time on its own thread (below;
`--lockstep`: inside the bridge's loop like Chrono) and drawn flat on the road. In lockstep the
car does what the NMPC predicts, so this shows the offline bicycle laps in the CARLA scenery
rather than testing the controllers against a different car; threaded, it also tests them against
their own computation time. The grass's
friction applies here too. The model's equations, tire curves, lags and limits:
[`docs/bicycle_plant.pdf`](docs/bicycle_plant.pdf).

The bicycle plant can also run in real time on its own thread (`BicyclePlant::start`, 1 kHz by
default), woken by a periodic kernel timer at absolute deadlines so the rate does not drift; a
late tick is caught up, so plant time stays wall-clock time. Its state is the global
`sim::bicycle_state` (`state()`, `bicycle_state[IUX]`, `command(u)`, `mu`), which controllers
read and command without ever blocking the plant. The thread asks for `SCHED_FIFO`, which needs
root, `CAP_SYS_NICE` or an rtprio limit (`ulimit -r`); otherwise it runs at normal priority
(`timing()` reports which, with the wake-up latency and overruns). The offline runs and the bridge
step the plant themselves, except the bridge with `--dynamics bicycle`.

With `--dynamics bicycle` the bridge runs three threads side by side in real time instead of one
loop: the plant at 1 kHz on that timer thread, the controller at `--control-rate` (100 Hz) (its solve releases
Python's GIL), and the CARLA world (draw the car, traffic and plan, camera, log, end of the lap)
at `--world-rate` (25 Hz). They share the state and the input through `trackctl.bicycle_state`, each taking the
latest value; nothing waits for anything else, and the rate panel shows the three rates as they
are. A slow solve therefore leaves the car on its previous input, which the lockstep runs (the
other dynamics, and `--lockstep`) hide by stopping the world during every solve. Pure pursuit laps
the same in both; the NMPC, whose solves take about 100 ms and which does not yet compensate for
that delay, does not keep the car on the track in real time.

`--controller nmpc_fallback` runs both controllers at once (`sim/racetrack/fallback_control.hpp`,
all in C++): the NMPC solving every `--nmpc-period` (200 ms), each solve planning from where the car
will be when that period is up (predicted with the bicycle model and the inputs the current plan
sends meanwhile), pure pursuit every 20 ms, and a command thread that every 20 ms sends the NMPC's
planned input for that moment, 10 of them per plan, until the next plan takes over. The plan's
steps match: 20 ms for the 0.2 s that is played, 0.1 s after that, out to a 2 s horizon (28 steps). A solve that
runs past its period is not dropped: pure pursuit is blended in until it arrives, then its plan takes
over from the input it planned for that moment and the next solve starts at once. A handover either
way is a ramp over `--blend` (200 ms) rather than a switch: the command is
(1 - w) u_NMPC + w u_pure_pursuit, w moving to 1 while the NMPC has no plan for the moment and back
to 0 once it has, the NMPC's part meanwhile its last plan carried on past its period. An input goes
out every 20 ms whatever the NMPC does. The HUD's banner says whose it is (green NMPC with the
plan's age, orange pure pursuit with the reason, yellow a blend with each one's share), the input
panel plots both controllers' inputs and the one sent, the rate panel shows all five threads, the
`.csv` has a `command` column and the `.json` the NMPC's and the blends' shares of the commands.

Measured against the stand-in CARLA on an 8-core VM: with the 100 ms deadline almost no solve
made it with IPOPT (about 9 of the 13 iterations it needed in 100 ms with everything else
running), so pure pursuit drove and lapped cleanly. The NMPC now solves with FATROP from a
time-shifted warm start at tolerance 1e-3: offline, p50 58 ms and p95 75 ms against IPOPT's 99 and
138 ms, the same lap (72.2 s against 72.3 s). With 200 ms the NMPC sends about half the
commands, but handing over between the two in the middle of a manoeuvre (different lines, pure
pursuit braking hard in a turn) has spun the car in 2 of 3 runs. With the 200 ms period and the
blended handover (CARLA server, default sensing), the NMPC drives 98% of the commands and the
hairpin handover no longer spins it, but the plan, now up to 400 ms old at the end of its period,
let the car weave while passing on the straight at s 510-590 m: 1 of 2 laps left the track there and
the other touched the edge (72.8 s); with `--nmpc-period 0.1 --control-rate 100` the lap was clean
(72.4 s, asphalt margin 0.17 m). With those 20 ms steps for the played 0.2 s and the 2 s horizon, the
solve takes p50 51 ms, p95 about 80 ms, all 361 of a lap within the 200 ms period, and both laps run
were clean of the weaving (72.0 s), though with the sideslip briefly past its 7° limit in most corners
(up to 8.7°) and the body 0.07 m over the asphalt's edge at s 638 m. The 2 s horizon makes the
NMPC oscillate on the straights; why, and the open work on it and on pure pursuit:
[`docs/todo.md`](docs/todo.md). How all of this was built and
measured: [`docs/realtime_control_report.md`](docs/realtime_control_report.md). Where the NMPC's
solve time goes and how to bring it under 50 ms without changing the lap (FATROP, compiled
functions, a warm-started barrier: 105 → 19 ms offline):
[`docs/nmpc_solve_time.pdf`](docs/nmpc_solve_time.pdf).

The controllers see the same thing in every case: the state at the centre of gravity in a
right-handed frame (`carla_bridge/frames.py` converts from CARLA's left-handed one) and the other
cars in track coordinates.

## Results (offline, the same controllers and plants without CARLA)

`./run.sh --offline` / `build/simulate`, default sensing (seed 0) unless marked:

| Plant | Controller | Lap | Speed | Passed | Clearance | Asphalt margin | Peak sideslip |
|---|---|---|---|---|---|---|---|
| bicycle | NMPC | 72.2 s | 44-70 km/h | 2/6 | 0.86 m | 0.15 m | 7.1° |
| bicycle | pure pursuit | 98.5 s | 24-61 km/h | 1/6 | 1.34 m | 0.11 m | 6.8° |
| Chrono | NMPC | 70.2 s | 49-72 km/h | 3/6 | 0.86 m | 0.19 m | 7.0° |
| Chrono | pure pursuit | 98.2 s | 24-61 km/h | 1/6 | 1.29 m | 0.19 m | 5.1° |
| Chrono, exact sensing | NMPC | 66.3 s | 50-80 km/h | 3/6 | 0.68 m | 0.11 m | 7.1° |

NMPC solves averaged 55-135 ms in these runs (several running at once on one machine), above the
50 ms control period; the simulation is synchronous, so this slows it down rather than changing it.
A few solves per lap on Chrono end without converging (11 in the default run); their result is
applied anyway.

#### Solve-time logs

Every NMPC run, offline (`build/simulate`) or in CARLA, also writes `<controller>_<dynamics>_solves.csv`.
It has one row per solve, with these columns:

| column | meaning |
|---|---|
| `t`, `s0`, `ux0` | time [s], path position [m] and measured speed [m/s] at the solve |
| `t_wall` | wall time around the solver call [s] (what `solve_mean` averages) |
| `t_total`, `t_proc` | the solver's wall and CPU time [s] (IPOPT's own timers; for FATROP, measured around the call) |
| `t_hess`, `t_jac`, `t_grad`, `t_f`, `t_g` | time in the Lagrangian Hessian, constraint Jacobian, cost gradient, cost and constraints [s] |
| `iterations`, `success`, `status` | solver iterations, convergence and return status |
| `warm` | started from the previous solution, shifted to this solve's time |
| `n_obstacles` | detected cars in the problem |
| `obj` | final cost |
| `slack_obstacle`, `slack_edge`, `slack_sideslip`, `slack_speed` | largest slack of each soft constraint over the horizon |
| `x0_gap` | distance between the shifted previous plan's first state and the measured one |
| `directive` | a supervisor's directive (track_control/README.md) was in force |

The result `.json` adds these keys next to `solve_mean`/`solve_max`/`not_converged`:
- `solve_p50` and `solve_p95`
- `overrun_fraction`: the share of solves longer than the control period
- `iter_mean` and `iter_max`
- `time_share`: hess, jac, grad, f, g and `ipopt_internal` (the solver's own work: linear solves and the rest, whichever solver) as fractions of the solver's time
- `proc_over_wall`: CPU over wall time. Well below 1 means the solver was waiting for a CPU; above 1 means threaded linear algebra.
- `status_counts`
- `first_solve`: the cold start

The run summary prints them as a second line.

### Making the NMPC drive the Chrono car

Planning with the parameters first identified for the Chrono sedan, the NMPC spun it in the first
chicane. Two model errors, both found by measuring the Chrono car:

1. **Tires, per axle.** Steering ramps at 10-22 m/s give each axle's lateral force against its
   slip angle. One friction coefficient for both axles had made the model understeer at the limit;
   Chrono's sedan is close to neutral, its rear peaking lower (1.04 Fz) than its front (1.15 Fz).
   The model now has a peak friction per axle (`grip_f`, `grip_r`), fitted by
   `scripts/fit_chrono_tires.py` (RMS error 134-174 N per axle, from 226-535 N).
2. **Drive force as a yaw control.** The model's friction circle says front-wheel drive force
   takes lateral grip from the front tires; Chrono's tires lose much less. The NMPC was correcting
   oversteer with full throttle, which does not work on the Chrono car. Planning with a quarter of
   the drive force (`config/tuning.json`) removes that lever: clean laps on three noise seeds
   (70.2-70.6 s, sideslip within 7°, 0.19 m or more from the asphalt edge).

## Layout

```
track_control/            the controller library (standalone CMake project; see its README)
  include/track_control/    path.hpp, controller.hpp, pure_pursuit.hpp, nmpc.hpp, speed_profile.hpp,
                            vehicle_params.hpp, types.hpp, model/ (bicycle model, Fiala tire)
  tests/ examples/
sim/                      offline simulation, shared with the CARLA bridge
  plant/                    bicycle plant (model + lags), Project Chrono plant, identify_chrono
  racetrack/                the circuit, perception (noise, range), the driver glue, the lap runner
  simulate.cpp              build/simulate
bindings/trackctl.cpp     Python module (controllers, track, plants) for the bridge
carla_bridge/             CARLA side: OpenDRIVE world, scenery, frames, actuation, dynamics backends,
                            camera/HUD/video
config/                   racetrack.json, vehicles/*.json, tuning.json (NMPC margins per dynamics)
scripts/                  install_chrono.sh, install_carla.sh, start_carla.sh, fit_chrono_tires.py
tests/                    the bridge against a stand-in carla module (tests/fake_carla)
setup.sh, run.sh
```

## Tests

```bash
ctest --test-dir build                       # the library: paths, model, controllers in closed loop;
                                             # the bicycle plant's real-time thread
.venv/bin/python -m pytest -m "not slow"     # the bridge: OpenDRIVE geometry, frames, actuation, video
.venv/bin/python -m pytest -m slow           # whole laps through the bridge against the stand-in CARLA
```

The stand-in `carla` module (`tests/fake_carla/carla.py`) follows CARLA's conventions
(left-handed frame, degrees, actor origin not centre of gravity, pedal and steering inputs, an
engine that delivers 85 % of what the throttle asks) with the bicycle model as its physics, so a
wrong sign or offset anywhere in the bridge fails a lap.

## Status: what has and has not been run

- Run and checked here: the library and offline simulation on both plants (results above; the
  bicycle-plant laps are identical, sample for sample, to the project this was refactored from),
  the Python module, and the whole bridge against the stand-in CARLA, including video writing.
  `setup.sh --no-carla` was run from scratch in a clean Ubuntu 22.04 container, Chrono build
  included, followed by an offline NMPC lap on Chrono and laps through the bridge.
- Chrono is compiled with `-march=native`, so its results differ slightly between machines and
  compilers: the exact-sensing NMPC lap on Chrono was 66.3 s here (GCC 13) and 66.4 s in the
  container (GCC 11), keeping 0.11 m and 0.08 m from the asphalt edge.
- **Not run: a real CARLA server.** Nothing here has driven CARLA yet. What may need adjusting the
  first time:
  - the server's start on a headless cloud GPU (Vulkan): `scripts/start_carla.sh` shows the log;
  - the actuation loop's gains and the Model 3's assumed parameters, which decide how well the
    NMPC's model matches CARLA's car; the NMPC margins for CARLA in `config/tuning.json` are a
    cautious guess, not tuned;
  - that CARLA reports a vehicle's velocity at the actor origin (the frames conversion assumes so;
    the difference is yaw rate x 0.45 m in lateral velocity);
  - the sign of the Chrono car's roll as drawn in CARLA (visual only).

## Using the library elsewhere

`track_control/` has its own CMakeLists, install rules and package config, and no dependency on
CARLA, Chrono or Python. To publish it on its own with its history:
`git subtree split --prefix=track_control -b track_control` and push that branch to a new
repository. For a CARLA project, the bridge here shows the three conversions you need: state
(`frames.py`), inputs (`actuation.py`) and the route (`opendrive.py` for a generated track, or
`tc::Path::from_waypoints` for CARLA's own waypoints).

## License

MIT (see LICENSE). CARLA, Project Chrono, CasADi and IPOPT have their own licenses.
