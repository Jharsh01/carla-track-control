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
"""
import argparse
import csv
import json
import math
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "build", "python"))

import numpy as np  # noqa: E402

import trackctl  # noqa: E402

VEHICLE_FILES = {"carla": "model3.json", "chrono": "chrono_sedan.json", "bicycle": "model3.json"}
# columns of <stem>_solves.csv, one row per NMPC solve (NMPC::SolveStats; build/simulate writes the same)
SOLVE_COLUMNS = ["t", "s0", "ux0", "t_wall", "t_total", "t_proc", "t_hess", "t_jac", "t_grad", "t_f", "t_g",
                 "iterations", "success", "status", "warm", "n_obstacles", "obj", "slack_obstacle", "slack_edge",
                 "slack_sideslip", "slack_speed", "x0_gap"]


def parse(argv=None):
    p = argparse.ArgumentParser(prog="python -m carla_bridge", description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--controller", choices=["nmpc", "pure_pursuit", "both"], default="both")
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
    p.add_argument("--no-plan", action="store_true", help="do not draw the NMPC's predicted path in the world")
    p.add_argument("--no-scenery", action="store_true",
                   help="CARLA's plain generated road instead of asphalt, kerbs, grass, barriers and trees")
    return p.parse_args(argv)


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


def drive(args, client, circuit, controller, stream=None):
    import carla

    from . import view, world as cw

    dynamics_name = args.dynamics
    params = trackctl.VehicleParams.from_file(
        args.params or os.path.join(trackctl.config_dir, "vehicles", VEHICLE_FILES[dynamics_name]))
    perception = (trackctl.Perception.exact(circuit) if args.perfect else
                  trackctl.Perception.noisy(circuit, road_range=args.road_range, seed=args.seed))
    driver = trackctl.Driver(controller, circuit, params, perception, trackctl.load_tuning(dynamics_name))

    world, original_settings = cw.build_world(client, circuit, args.dt, scenery=not args.no_scenery)
    actors = []
    try:
        traffic = cw.Traffic(world, circuit)
        actors.append(traffic)
        dynamics = cw.DYNAMICS[dynamics_name](world, circuit, params, args.dt)
        actors.append(dynamics)
        traffic.place(0.0)
        camera = video = hud = rates = None
        width = 1280
        if not args.no_video or stream:
            camera = view.ChaseCamera(world, dynamics.actor)
            actors.append(camera)
            hud = view.Hud(circuit)
            if not args.no_rates:
                rates = view.RatePanel(args.dt)
                width += view.RatePanel.WIDTH
        os.makedirs(args.out, exist_ok=True)
        stem = os.path.join(args.out, "%s_%s" % (controller, dynamics_name))
        if not args.no_video:
            video = view.Video(stem + ".mp4", (width, 720), fps=round(1.0 / args.dt))

        def friction(x):
            return circuit.friction(circuit.track_state(np.asarray(x))[1])

        t = 0.0
        x = np.asarray(dynamics.state())
        run = Run(circuit, x)
        spectator = world.get_spectator()
        wall = time.time()
        print("driving %d lap(s) with %s on %s dynamics ..." % (args.laps, controller, dynamics_name))
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
            x = np.asarray(dynamics.state())
            # the spectator (what a CARLA window would show) follows the car too
            ego = dynamics.actor.get_transform()
            spectator.set_transform(carla.Transform(ego.transform(carla.Location(x=-12.0, z=6.0)),
                                                    carla.Rotation(pitch=-20.0, yaw=ego.rotation.yaw)))
            if camera:
                rgb = camera.read(frame)
                lines = view.hud_lines(controller, dynamics_name, t, x, e_y, run.progress,
                                       args.laps * circuit.track.length, driver.last_solve_time)
                traffic_xy = [(p.x, p.y) for p in (circuit.traffic_pose(i, t) for i in range(circuit.n_traffic))]
                rgb = hud.draw(rgb, lines, (x[0], x[1]), traffic_xy)
                if rates:
                    rgb = rates.draw(rgb)
                if video:
                    video.add(rgb)
                if stream:
                    stream.add(rgb)
        if video:
            video.close()

        result = run.summary(controller, dynamics_name, driver, args.dt)
        with open(stem + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "X", "Y", "psi", "Ux", "Uy", "r", "delta", "delta_rate", "Fx", "s", "e_y", "mu"])
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
                  "   time in hessian %.0f%%, jacobian %.0f%%, ipopt %.0f%%   cpu/wall %.2f   first solve %.0f ms"
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
