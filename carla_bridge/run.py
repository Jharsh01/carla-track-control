"""Drive the race track in CARLA with the NMPC and/or pure pursuit, on CARLA's, Project Chrono's or the bicycle model's dynamics.

    python -m carla_bridge                                  both controllers, CARLA dynamics
    python -m carla_bridge --controller nmpc --dynamics chrono
    python -m carla_bridge --stream 8080                    also watch live on http://localhost:8080
    python -m carla_bridge --perfect                        exact sensing (default: noisy, limited range)
    python -m carla_bridge --laps 2 --no-video

For each run: results/carla/<controller>_<dynamics>.mp4 (the chase camera with
a HUD), .csv (one row per step), .json (the result, as build/simulate
writes offline) and, for the NMPC, _solves.csv (one row per solve: timing
breakdown, iterations, status, slacks). The CARLA server must be running (scripts/start_carla.sh;
run.sh starts it).

With --dynamics bicycle, three threads run side by side in real time: the plant
at 1 kHz (C++, timer-driven), the controller at --control-rate (100 Hz), and the
CARLA world (draw, camera, log) at --world-rate (25 Hz). With --controller nmpc_fallback the
controller is three threads (C++, sim/racetrack/fallback_control.hpp): the NMPC
solving every --nmpc-period (200 ms), each plan played for one period (10
commands at 50 Hz), pure pursuit at --control-rate, and the command thread
sending the NMPC's planned input while it has a plan for the moment and pure
pursuit's otherwise, ramping from one to the other over --blend seconds; a late
solve is not dropped, its plan takes over when it arrives. The HUD says whose input it is. They share the state and the input through the
global trackctl.bicycle_state, each taking the latest value: nothing waits for
an NMPC solve, so a slow solve leaves the car on its last input. --lockstep
runs them one after the other instead, as the other dynamics do.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "build", "python"))

import numpy as np  # noqa: E402

import trackctl  # noqa: E402

VEHICLE_FILES = {"carla": "model3.json", "chrono": "chrono_sedan.json", "bicycle": "model3.json"}
# columns of <stem>_solves.csv, one row per NMPC solve (NMPC::SolveStats; build/simulate writes the same)
SOLVE_COLUMNS = ["t", "s0", "ux0", "t_wall", "t_total", "t_proc", "t_hess", "t_jac", "t_grad", "t_f", "t_g",
                 "iterations", "success", "status", "warm", "n_obstacles", "obj", "slack_obstacle", "slack_edge",
                 "slack_sideslip", "slack_speed", "x0_gap", "directive"]


def parse(argv=None):
    p = argparse.ArgumentParser(prog="python -m carla_bridge", description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--controller", choices=["nmpc", "pure_pursuit", "both", "nmpc_fallback"], default="both",
                   help="nmpc_fallback (threaded bicycle only): the NMPC and pure pursuit both running, the input "
                        "blended to pure pursuit's whenever the NMPC has no plan for the moment")
    p.add_argument("--control-rate", type=float,
                   help="threaded: rate at which an input is commanded [Hz] (pure pursuit computes at it too); "
                        "default 50 with nmpc_fallback, otherwise 100")
    p.add_argument("--nmpc-period", "--nmpc-deadline", type=float, default=0.2, dest="nmpc_period",
                   help="nmpc_fallback: an NMPC solve every this many seconds, its plan starting a period later and "
                        "played for one period; a solve past it is kept, pure pursuit blended in until it arrives")
    p.add_argument("--nmpc-dt", type=float, default=0.0,
                   help="the NMPC's step [s], after the fine ones (0: its default, 0.1 s)")
    p.add_argument("--nmpc-horizon", type=float,
                   help="how far the NMPC plans [s]; default 2 with nmpc_fallback, otherwise the NMPC's (4)")
    p.add_argument("--nmpc-fine-dt", type=float,
                   help="a finer NMPC step for the first --nmpc-fine seconds [s]; default with nmpc_fallback one "
                        "command (1 / --control-rate) for one --nmpc-period, the part of each plan played; "
                        "otherwise none")
    p.add_argument("--nmpc-fine", type=float, help="how long the fine steps last [s]")
    p.add_argument("--blend", type=float, default=0.2,
                   help="nmpc_fallback: seconds a handover between the NMPC and pure pursuit takes (0: a switch)")
    p.add_argument("--dynamics", choices=["carla", "chrono", "bicycle"], default="carla",
                   help="who simulates the ego car: CARLA's vehicle physics, the Project Chrono sedan or the "
                        "controllers' own bicycle model (the offline plant)")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--dt", type=float, default=0.05, help="world step = control period [s]")
    p.add_argument("--laps", type=int, default=1)
    p.add_argument("--t-max", type=float, default=240.0, help="give up after this long [s]")
    p.add_argument("--track", default=os.path.join(REPO, "config", "racetrack.json"))
    p.add_argument("--params", help="vehicle parameters for the controllers (default: config/vehicles/ for the dynamics)")
    p.add_argument("--perfect", action="store_true", help="exact traffic and ego state, the whole road known")
    p.add_argument("--road-range", type=float, default=40.0, help="how far ahead the road is known [m]")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=os.path.join(REPO, "results", "carla"))
    p.add_argument("--no-video", action="store_true", help="no camera: fastest; logs only")
    p.add_argument("--stream", type=int, metavar="PORT", help="serve the camera view live on this port")
    p.add_argument("--no-rates", action="store_true",
                   help="no side panel plotting the controller's and the plant's average rate in wall-clock Hz")
    p.add_argument("--no-inputs", action="store_true",
                   help="no side panel plotting the controllers' inputs (acceleration and steering rate)")
    p.add_argument("--no-plan", action="store_true", help="do not draw the NMPC's predicted path in the world")
    p.add_argument("--lockstep", action="store_true",
                   help="with --dynamics bicycle: one loop, controller then plant then CARLA tick, CARLA waiting "
                        "for each (the other dynamics always run so); default: plant (1 kHz), controller and "
                        "CARLA world on their own threads in real time")
    p.add_argument("--plant-rate", type=float, default=1000.0, help="the threaded bicycle plant's rate [Hz]")
    p.add_argument("--world-rate", type=float, default=25.0,
                   help="threaded: rate of the CARLA world thread [Hz]: ticks, camera frames, video, log")
    p.add_argument("--no-scenery", action="store_true",
                   help="CARLA's plain generated road instead of asphalt, kerbs, grass, barriers and trees")
    args = p.parse_args(argv)
    fallback = args.controller == "nmpc_fallback"
    if args.control_rate is None:
        args.control_rate = 50.0 if fallback else 100.0
    if args.nmpc_horizon is None:
        args.nmpc_horizon = 2.0 if fallback else 0.0
    if args.nmpc_fine_dt is None:
        args.nmpc_fine_dt = 1.0 / args.control_rate if fallback else 0.0
    if args.nmpc_fine is None:
        args.nmpc_fine = args.nmpc_period if fallback else 0.0
    return args


class Run:
    """The quantities build/simulate reports, accumulated step by step."""

    def __init__(self, circuit, x0):
        self.circuit = circuit
        self.track = circuit.track
        self.s_prev = self.track.project(x0[0], x0[1])[0]
        self.progress = 0.0
        self.min_clearance = math.inf
        self.road_margin = math.inf
        self.time_on_grass = 0.0
        self.peak_sideslip = 0.0
        self.min_speed, self.max_speed = math.inf, -math.inf
        self.collided = self.left_track = False
        self.lap_time = None
        self.rows = []

    def update(self, t, x, u, dt, laps):
        s, e_y = self.track.project(x[0], x[1])
        self.progress += self.track.difference(s, self.s_prev)
        self.s_prev = s
        mu = self.circuit.friction(e_y)
        clearance = self.circuit.clearance(t, x)
        self.min_clearance = min(self.min_clearance, clearance)
        self.road_margin = min(self.road_margin, self.circuit.road_margin(x))
        if mu == self.circuit.mu_grass:
            self.time_on_grass += dt
        self.peak_sideslip = max(self.peak_sideslip, abs(math.atan2(x[4], x[3])))
        self.min_speed, self.max_speed = min(self.min_speed, x[3]), max(self.max_speed, x[3])
        self.rows.append([t] + list(x) + list(u) + [self.progress, e_y, mu])
        if clearance <= 0.0:
            self.collided = True
        elif math.isnan(mu):
            self.left_track = True
        elif self.progress >= laps * self.track.length:
            self.lap_time = t
        return e_y

    @property
    def done(self):
        return self.collided or self.left_track or self.lap_time is not None

    def summary(self, controller, dynamics, driver, dt):
        out = {"controller": controller, "dynamics": dynamics, "completed": self.lap_time is not None,
               "lap_time": self.lap_time, "collided": self.collided, "left_track": self.left_track,
               "min_clearance": self.min_clearance, "road_margin": self.road_margin,
               "time_on_grass": self.time_on_grass, "peak_sideslip": self.peak_sideslip,
               "min_speed": self.min_speed, "max_speed": self.max_speed}
        times = driver.solve_times
        if times:
            out.update(solve_mean=float(np.mean(times)), solve_max=float(np.max(times)), not_converged=driver.failures)
            out.update(solve_breakdown(getattr(driver, "solve_stats", []), dt))
        return out


def solve_breakdown(stats, dt):
    """Where the NMPC's solve time went, over all solves of a run: the summary keys next to solve_mean."""
    if not stats:
        return {}
    wall = np.array([s["t_wall"] for s in stats])
    total = sum(s["t_total"] for s in stats)
    iters = np.array([s["iterations"] for s in stats])
    share = {k: sum(s["t_" + k] for s in stats) / total if total > 0 else 0.0
             for k in ("hess", "jac", "grad", "f", "g")}
    share["ipopt_internal"] = 1.0 - sum(share.values()) if total > 0 else 0.0  # linear solves and the rest
    statuses = {}
    for s in stats:
        statuses[s["status"]] = statuses.get(s["status"], 0) + 1
    return {"solve_p50": float(np.percentile(wall, 50)), "solve_p95": float(np.percentile(wall, 95)),
            "overrun_fraction": float(np.mean(wall > dt)), "iter_mean": float(np.mean(iters)),
            "iter_max": int(np.max(iters)), "time_share": share,
            "proc_over_wall": sum(s["t_proc"] for s in stats) / total if total > 0 else 0.0,
            "status_counts": statuses, "first_solve": float(wall[0])}


def write_solves(path, stats):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SOLVE_COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(stats)


def name_os_thread(name):
    """Give the calling thread `name` where top -H and the like show it (Linux; up to 15 characters)."""
    try:
        import ctypes

        ctypes.CDLL(None).prctl(15, name.encode()[:15], 0, 0, 0)  # PR_SET_NAME
    except (OSError, AttributeError):
        pass


class ViewThread(threading.Thread):
    """Draws the HUD and rate panel on the camera frames and writes them to the video and stream, off the CARLA
    world thread: that thread hands over its latest frame and goes on. A frame the view has no time for is
    skipped, and the video repeats the last one drawn in its place, so the video keeps real-time pace."""

    def __init__(self, compose, publish):
        super().__init__(name="view", daemon=True)
        self.compose, self.publish = compose, publish
        self.ready = threading.Condition()
        self.latest = None  # (world tick, compose arguments)
        self.closing = False
        self.written = 0  # world ticks the video has frames for
        self.drawn = 0  # frames composed
        self.error = None

    def submit(self, tick, item):
        with self.ready:
            self.latest = (tick, item)
            self.ready.notify()

    def run(self):
        name_os_thread("view")
        try:
            while True:
                with self.ready:
                    self.ready.wait_for(lambda: self.latest is not None or self.closing)
                    if self.latest is None:
                        return
                    (tick, item), self.latest = self.latest, None
                rgb = self.compose(*item)
                self.drawn += 1
                self.publish(rgb, frames=tick - self.written)
                self.written = tick
        except Exception as error:  # noqa: BLE001 - handed to the world thread, which stops the run
            self.error = error

    def finish(self):
        """Draw the last frame handed over, then stop."""
        with self.ready:
            self.closing = True
            self.ready.notify()
        self.join()


class ControllerThread(threading.Thread):
    """The controller on its own thread: every `period` seconds of wall time (at once after a solve that took
    longer) it reads the plant's latest state from trackctl.bicycle_state, solves, and commands the input
    there. The solve releases the GIL, so the CARLA thread runs meanwhile."""

    def __init__(self, driver, period, keep_plan):
        super().__init__(name="controller", daemon=True)
        self.driver, self.period, self.keep_plan = driver, period, keep_plan
        self.halt = threading.Event()
        self.solves = 0  # completed
        self.last_solve = None  # [s]
        self.plan = []  # the NMPC's latest predicted path, copied here so the CARLA thread can draw it
        self.error = None

    def run(self):
        name_os_thread("controller")
        try:
            next_time = time.perf_counter()
            while not self.halt.is_set():
                t, x = trackctl.bicycle_state.state()
                start = time.perf_counter()
                u = self.driver(t, x)
                self.last_solve = time.perf_counter() - start
                trackctl.bicycle_state.command(u)
                if self.keep_plan:
                    self.plan = self.driver.plan
                self.solves += 1
                next_time = max(next_time + self.period, time.perf_counter())
                self.halt.wait(next_time - time.perf_counter())
        except Exception as error:  # noqa: BLE001 - handed to the CARLA thread, which stops the run
            self.error = error

    def stop(self):
        self.halt.set()
        self.join()


def drive(args, client, circuit, controller, stream=None):
    import carla

    from . import view, world as cw

    dynamics_name = args.dynamics
    params = trackctl.VehicleParams.from_file(
        args.params or os.path.join(trackctl.config_dir, "vehicles", VEHICLE_FILES[dynamics_name]))
    perception = (trackctl.Perception.exact(circuit) if args.perfect else
                  trackctl.Perception.noisy(circuit, road_range=args.road_range, seed=args.seed))
    tuning = trackctl.load_tuning(dynamics_name)
    tuning.nmpc_dt, tuning.nmpc_horizon = args.nmpc_dt, args.nmpc_horizon
    tuning.nmpc_fine_dt, tuning.nmpc_fine = args.nmpc_fine_dt, args.nmpc_fine
    fallback_driver = None
    if controller == "nmpc_fallback":
        if dynamics_name != "bicycle" or args.lockstep:
            raise SystemExit("--controller nmpc_fallback needs --dynamics bicycle without --lockstep")
        driver = trackctl.Driver("nmpc", circuit, params, perception, tuning)
        fallback_perception = (trackctl.Perception.exact(circuit) if args.perfect else
                               trackctl.Perception.noisy(circuit, road_range=args.road_range, seed=args.seed))
        fallback_driver = trackctl.Driver("pure_pursuit", circuit, params, fallback_perception)
    else:
        driver = trackctl.Driver(controller, circuit, params, perception, tuning)

    threaded = dynamics_name == "bicycle" and not args.lockstep
    world_dt = 1.0 / args.world_rate if threaded else args.dt  # one world tick (and camera frame)
    world, original_settings = cw.build_world(client, circuit, world_dt, scenery=not args.no_scenery)
    actors = []
    try:
        traffic = cw.Traffic(world, circuit)
        actors.append(traffic)
        dynamics = (cw.ThreadedBicycleDynamics if threaded else cw.DYNAMICS[dynamics_name])(
            world, circuit, params, world_dt)
        actors.append(dynamics)
        traffic.place(0.0)
        camera = video = hud = rates = inputs = None
        width = 1280
        if not args.no_video or stream:
            camera = view.ChaseCamera(world, dynamics.actor)
            actors.append(camera)
            hud = view.Hud(circuit)
            if not args.no_inputs:
                inputs = view.InputPanel(params, ["nmpc", "pure_pursuit", "command"] if fallback_driver else [controller])
                width += view.InputPanel.WIDTH
            if not args.no_rates:
                if fallback_driver:
                    rates = view.RatePanel(world_dt, series=view.RatePanel.FALLBACK, real_time={
                        "command": args.control_rate, "nmpc": 1.0 / args.nmpc_period,
                        "pure_pursuit": args.control_rate, "plant": args.plant_rate})
                elif threaded:
                    rates = view.RatePanel(world_dt, series=view.RatePanel.THREADS,
                                           real_time={"controller": args.control_rate, "plant": args.plant_rate})
                else:
                    rates = view.RatePanel(args.dt)
                width += view.RatePanel.WIDTH
        os.makedirs(args.out, exist_ok=True)
        stem = os.path.join(args.out, "%s_%s" % (controller, dynamics_name))
        if not args.no_video:
            video = view.Video(stem + ".mp4", (width, 720), fps=round(1.0 / world_dt))

        def friction(x):
            return circuit.friction(circuit.track_state(np.asarray(x))[1])

        spectator = world.get_spectator()

        def capture(frame):
            """The spectator (what a CARLA window would show) behind the car; the camera's image of `frame`."""
            ego = dynamics.actor.get_transform()
            spectator.set_transform(carla.Transform(ego.transform(carla.Location(x=-12.0, z=6.0)),
                                                    carla.Rotation(pitch=-20.0, yaw=ego.rotation.yaw)))
            return camera.read(frame) if camera else None

        def compose(rgb, t, x, e_y, progress, solve_time, banner=None):
            """The camera image with the HUD and the rate panel."""
            lines = view.hud_lines(controller, dynamics_name, t, x, e_y, progress,
                                   args.laps * circuit.track.length, solve_time)
            traffic_xy = [(p.x, p.y) for p in (circuit.traffic_pose(i, t) for i in range(circuit.n_traffic))]
            rgb = hud.draw(rgb, lines, (x[0], x[1]), traffic_xy, banner)
            if inputs:
                rgb = inputs.draw(rgb)
            return rates.draw(rgb) if rates else rgb

        def publish(rgb, frames=1):
            """A composed image to the video (as `frames` frames) and the live stream."""
            if video:
                for _ in range(frames):
                    video.add(rgb)
            if stream:
                stream.add(rgb)

        def show(frame, t, x, e_y, progress, solve_time):
            """Lockstep: capture, compose and publish in this thread."""
            rgb = capture(frame)
            if rgb is not None:
                publish(compose(rgb, t, x, e_y, progress, solve_time))

        run = Run(circuit, np.asarray(dynamics.state()))
        wall = time.time()
        print("driving %d lap(s) with %s on %s dynamics%s ..." % (
            args.laps, controller, dynamics_name, " (threads: plant %.0f Hz, controller %.0f Hz, world %.0f Hz)" % (
                args.plant_rate, args.control_rate, 1.0 / world_dt) if threaded else ""))
        extra = {}
        if threaded:
            viewer = ViewThread(compose, publish) if camera else None
            if viewer:
                viewer.start()
            try:
                extra = run_threads(args, world, circuit, controller, driver, dynamics, traffic, run, rates, inputs,
                                    capture, viewer, fallback_driver, world_dt)
            finally:
                if viewer:
                    viewer.finish()
        else:
            run_lockstep(args, world, circuit, controller, driver, dynamics, traffic, run, rates, inputs, show,
                         friction)
        if video:
            video.close()

        result = run.summary(controller, dynamics_name, driver, args.dt)
        result.update(extra)
        with open(stem + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "X", "Y", "psi", "Ux", "Uy", "r", "delta", "delta_rate", "Fx", "s", "e_y", "mu"] +
                       (["command"] if fallback_driver else []))
            w.writerows(run.rows)
        stats = getattr(driver, "solve_stats", [])
        if stats:
            write_solves(stem + "_solves.csv", stats)
        with open(stem + ".json", "w") as f:
            json.dump(result, f, indent=2)
        report(result, time.time() - wall)
        if video:
            print("wrote %s.mp4" % stem)
        return result
    finally:
        for actor in reversed(actors):
            try:
                actor.destroy()
            except RuntimeError:
                pass
        world.apply_settings(original_settings)


def run_lockstep(args, world, circuit, controller, driver, dynamics, traffic, run, rates, inputs, show, friction):
    """One loop: the controller, then the plant through the world step, then CARLA's tick; CARLA waits for both."""
    t = 0.0
    x = np.asarray(dynamics.state())
    while t < args.t_max:
        start = time.perf_counter()
        u = np.asarray(driver(t, x))
        controller_time = time.perf_counter() - start
        e_y = run.update(t, x, u, args.dt, args.laps)
        if run.done:
            break
        start = time.perf_counter()
        dynamics.apply(u, x, friction)
        plant_time = time.perf_counter() - start
        traffic.place(t + args.dt)
        if not args.no_plan and controller == "nmpc":
            draw_plan(world, circuit, driver.plan, args.dt)
        start = time.perf_counter()
        frame = world.tick()
        dynamics.after_tick()
        plant_time += time.perf_counter() - start
        t += args.dt
        if rates:
            rates.add(t, controller_time, plant_time)
        if inputs:
            inputs.add(t, {controller: u})
        x = np.asarray(dynamics.state())
        show(frame, t, x, e_y, run.progress, driver.last_solve_time)


def run_threads(args, world, circuit, controller, driver, dynamics, traffic, run, rates, inputs, capture, viewer,
                fallback_driver=None, world_dt=0.04):
    """The plant (its own C++ thread), the controller (a Python thread; or, with `fallback_driver`, the NMPC, pure
    pursuit and the command thread in C++) and the CARLA world (this thread), each at its own rate in real time,
    sharing the state and input through trackctl.bicycle_state. This thread ticks CARLA every `world_dt` of wall time
    with the car where the plant has it, reads the camera, logs and ends the run; `viewer` (a ViewThread) draws the
    HUD and panel and writes the video and stream, so drawing never slows the world. Returns what goes into the result besides Run's summary."""
    from . import view

    x0 = np.asarray(dynamics.state())
    u0 = np.asarray(driver(0.0, x0))  # the first solve (the NMPC's is slow) before the clocks start
    keep_plan = not args.no_plan and controller in ("nmpc", "nmpc_fallback")
    if fallback_driver:
        u0 = np.asarray(fallback_driver(0.0, x0))  # pure pursuit drives until the NMPC's first plan
        control = trackctl.FallbackControl(driver, fallback_driver, rate=args.control_rate, period=args.nmpc_period,
                                           blend=args.blend)
    else:
        control = ControllerThread(driver, 1.0 / args.control_rate, keep_plan)
    dynamics.start(u0, rate=args.plant_rate)
    control.start()
    start = time.perf_counter()
    next_time = start
    ticks = 0
    t_prev = 0.0
    status = None
    try:
        while True:
            t, x = dynamics.sample()
            x = np.asarray(x)
            if not fallback_driver and control.error:
                raise control.error
            u = np.asarray(trackctl.bicycle_state.applied)
            e_y = run.update(t, x, u, t - t_prev, args.laps)
            t_prev = t
            banner = None
            if fallback_driver:
                status = control.status()
                run.rows[-1].append(status["source"])
                plan, solve_time = control.plan, status["last_solve"] or None
                banner = view.command_banner(status, args.nmpc_period)
                if inputs:
                    inputs.add(t, {"nmpc": status["nmpc_input"], "pure_pursuit": status["pure_pursuit_input"],
                                   "command": u}, status["source"])
            else:
                plan, solve_time = control.plan, control.last_solve
                if inputs:
                    inputs.add(t, {controller: u})
            if run.done or t >= args.t_max:
                break
            dynamics.after_tick()  # the car where the plant has it now
            traffic.place(t)
            if keep_plan and plan:
                draw_plan(world, circuit, plan, world_dt)
            frame = world.tick()
            ticks += 1
            if rates:
                wall = time.perf_counter() - start
                plant_ticks = dynamics.plant.timing()["ticks"]
                if fallback_driver:
                    rates.add_counts(t, {"command": (status["ticks"], wall), "nmpc": (status["solves"], wall),
                                         "pure_pursuit": (status["pure_pursuit_updates"], wall),
                                         "plant": (plant_ticks, wall), "world": (ticks, wall)})
                else:
                    rates.add_counts(t, {"controller": (control.solves, wall), "plant": (plant_ticks, wall),
                                         "world": (ticks, wall)})
            rgb = capture(frame)
            if viewer and rgb is not None:
                if viewer.error:
                    raise viewer.error
                viewer.submit(ticks, (rgb, t, x, e_y, run.progress, solve_time, banner))
            next_time = max(next_time + world_dt, time.perf_counter())
            time.sleep(max(0.0, next_time - time.perf_counter()))
    finally:
        control.stop()
        dynamics.stop()
    wall = time.perf_counter() - start
    timing = dynamics.plant.timing()
    if fallback_driver:
        status = control.status()
        commands = "command %.1f Hz (NMPC %d, pure pursuit %d, blended %d), NMPC %.1f solves/s (%d of %d on time), " \
                   "pure pursuit %.1f Hz" % (status["ticks"] / wall, status["nmpc_ticks"], status["pure_pursuit_ticks"],
                                            status["blend_ticks"], status["solves"] / wall, status["solves_on_time"],
                                            status["solves"], status["pure_pursuit_updates"] / wall)
    else:
        commands = "controller %.1f Hz" % (control.solves / wall)
    drawn = ", view %.1f Hz (%d of %d frames drawn, the others repeated in the video)" % (
        viewer.drawn / wall, viewer.drawn, ticks) if viewer else ""
    print("threads over %.1f s: plant %.1f Hz (%d overruns, wake-up latency mean %.0f us, max %.0f us, %s), "
          "%s, CARLA world %.1f Hz%s" % (
              wall, timing["ticks"] / wall, timing["overruns"], 1e6 * timing["latency_mean"],
              1e6 * timing["latency_max"], "SCHED_FIFO" if timing["realtime"] else "normal priority",
              commands, ticks / wall, drawn))
    if not fallback_driver:
        return {}
    ticks_total = max(1, status["ticks"])
    return {"nmpc_command_share": status["nmpc_ticks"] / ticks_total,
            "blend_command_share": status["blend_ticks"] / ticks_total, "nmpc_solves": status["solves"],
            "nmpc_solves_on_time": status["solves_on_time"], "nmpc_period": args.nmpc_period, "blend": args.blend,
            "control_rate": args.control_rate}


def draw_plan(world, circuit, plan, dt):
    """The NMPC's predicted path, as a green line on the road for one step."""
    import carla

    from . import frames

    points = []
    for s, e_y in plan:
        p = circuit.track.pose(s, e_y)
        cx, cy = frames.to_carla_xy(p.x, p.y)
        points.append(carla.Location(x=cx, y=cy, z=0.3))
    for a, b in zip(points[:-1], points[1:]):
        world.debug.draw_line(a, b, thickness=0.08, color=carla.Color(40, 220, 90), life_time=1.5 * dt)


def report(r, wall):
    outcome = "%6.1f s" % r["lap_time"] if r["completed"] else "   none"
    line = ("%-13s %-7s lap %s   speed %2.0f-%2.0f km/h   clearance %5.2f m   asphalt margin %5.2f m"
            "   on grass %3.1f s   sideslip %4.1f deg"
            % (r["controller"], r["dynamics"], outcome, 3.6 * r["min_speed"], 3.6 * r["max_speed"],
               r["min_clearance"], r["road_margin"], r["time_on_grass"], math.degrees(r["peak_sideslip"])))
    if "solve_mean" in r:
        line += "   | solve mean %.0f ms, max %.0f ms, %d not converged" % (
            1e3 * r["solve_mean"], 1e3 * r["solve_max"], r["not_converged"])
    detail = None
    if "time_share" in r:
        share = r["time_share"]
        detail = ("%-13s solve p50 %.0f ms, p95 %.0f ms, over dt %.0f%%   iterations mean %.1f, max %d"
                  "   time in hessian %.0f%%, jacobian %.0f%%, solver %.0f%%   cpu/wall %.2f   first solve %.0f ms"
                  % ("", 1e3 * r["solve_p50"], 1e3 * r["solve_p95"], 100 * r["overrun_fraction"], r["iter_mean"],
                     r["iter_max"], 100 * share["hess"], 100 * share["jac"], 100 * share["ipopt_internal"],
                     r["proc_over_wall"], 1e3 * r["first_solve"]))
    if r["collided"] or r["left_track"]:
        line += "   %s" % ("collision" if r["collided"] else "left the track")
    print(line + "   (%.0f s wall time)" % wall)
    if detail:
        print(detail)


def main(argv=None):
    args = parse(argv)
    from . import world as cw

    circuit = trackctl.Circuit(args.track)
    client = cw.connect(args.host, args.port)
    print("CARLA server %s, client %s" % (client.get_server_version(), client.get_client_version()))
    controllers = ["nmpc", "pure_pursuit"] if args.controller == "both" else [args.controller]
    stream = None
    if args.stream:
        from .view import Stream

        stream = Stream(args.stream)  # one server for all the runs: the page keeps showing the current one
        print("live view on http://localhost:%d/" % args.stream)
    try:
        results = [drive(args, client, circuit, c, stream) for c in controllers]
    finally:
        if stream:
            stream.close()
    return 0 if all(r["completed"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
