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

## Quick start on NVIDIA Brev

1. Create a GPU instance in **VM mode** (CARLA refuses to run as root, and VM mode gives you the
   `ubuntu` user), Ubuntu 22.04, a GPU with 16 GB or more (L4, A10G), about 60 GB of disk.
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
| `--dynamics carla\|chrono` | who simulates the ego car (default carla) |
| `--perfect` | exact sensing, the whole road known |
| `--road-range M`, `--seed N` | how far ahead the road is known; the noise seed |
| `--laps N`, `--t-max S` | |
| `--stream PORT`, `--no-video` | live view; no camera at all (fastest) |
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

The controllers see the same thing in both cases: the state at the centre of gravity in a
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
carla_bridge/             CARLA side: OpenDRIVE world, frames, actuation, dynamics backends, camera/HUD/video
config/                   racetrack.json, vehicles/*.json, tuning.json (NMPC margins per dynamics)
scripts/                  install_chrono.sh, install_carla.sh, start_carla.sh, fit_chrono_tires.py
tests/                    the bridge against a stand-in carla module (tests/fake_carla)
setup.sh, run.sh
```

## Tests

```bash
ctest --test-dir build                       # the library: paths, model, controllers in closed loop
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
