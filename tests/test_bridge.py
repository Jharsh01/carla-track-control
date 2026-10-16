"""The CARLA bridge without a CARLA server: OpenDRIVE geometry, frame conversions, actuation, and whole
laps through a stand-in `carla` module (tests/fake_carla) whose car follows CARLA's conventions."""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "fake_carla"))  # before the real client, which needs a server
sys.path.insert(0, os.path.join(REPO, "build", "python"))
sys.path.insert(0, REPO)

import carla  # noqa: E402  (the stand-in)
import trackctl  # noqa: E402

from carla_bridge import frames, opendrive  # noqa: E402
from carla_bridge.actuation import Actuation  # noqa: E402

assert carla.__file__.startswith(os.path.join(HERE, "fake_carla")), "the real carla module was imported"

CIRCUIT = trackctl.Circuit()


def test_opendrive_follows_the_track():
    xodr = opendrive.build(CIRCUIT)
    root = ET.fromstring(xodr)
    road = root.find("road")
    assert float(road.get("length")) == pytest.approx(CIRCUIT.track.length, abs=1e-6)
    # every point of the reference line lies on the track's centreline, with its heading
    for x, y, hdg in opendrive.reference_line(xodr, step=0.5):
        s, e_y = CIRCUIT.track.project(x, y)
        assert abs(e_y) < 1e-6
        assert math.remainder(hdg - CIRCUIT.track.pose(s).psi, 2 * math.pi) == pytest.approx(0.0, abs=1e-6)
    # the lanes: asphalt then grass on each side
    lanes = {int(l.get("id")): float(l.find("width").get("a")) for l in road.iter("lane") if l.find("width") is not None}
    assert lanes == {2: CIRCUIT.grass_width, 1: CIRCUIT.lane_width, -1: CIRCUIT.lane_width, -2: CIRCUIT.grass_width}


def test_frames_round_trip():
    # a car at (10, 5) heading 30 deg left of x, moving 12 m/s forward and 0.5 m/s left, turning left at 0.2 rad/s
    X, Y, psi, Ux, Uy, r = 10.0, 5.0, math.radians(30.0), 12.0, 0.5, 0.2
    cg = 0.45
    ox, oy = frames.origin_from_cg(X, Y, psi, cg)
    # what CARLA would report for the actor origin
    loc = frames.to_carla_xy(ox, oy)
    yaw = frames.to_carla_yaw(psi)
    vel = frames.velocity_to_carla(psi, Ux, Uy - r * cg)
    x = frames.state_from_carla(loc, yaw, vel, -math.degrees(r), cg, delta=0.01)
    assert x == pytest.approx([X, Y, psi, Ux, Uy, r, 0.01], abs=1e-12)
    assert yaw < 0.0 and loc[1] < 0.0  # left of x is negative yaw and negative y in CARLA


def test_actuation_signs_and_limits():
    p = trackctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "model3.json"))
    a = Actuation(p, max_steer_deg=70.0, dt=0.05)
    throttle, brake, steer = a.command([0.4, 2000.0], 15.0)  # steer left, drive
    assert throttle > 0.0 and brake == 0.0 and steer < 0.0  # CARLA steers right for positive steer
    assert a.delta == pytest.approx(0.4 * 0.05)
    a = Actuation(p, max_steer_deg=70.0, dt=0.05)  # fresh: no feedback yet, the feedforward alone
    throttle, brake, steer = a.command([0.0, -3000.0], 15.0)
    assert throttle == 0.0 and brake == pytest.approx(3000.0 / p.F_brake_max)
    # the car then slows less than the model says: the loop brakes harder
    _, brake_again, _ = a.command([0.0, -3000.0], 15.0 - 0.5 * 3000.0 / p.m * 0.05)
    assert brake_again > brake
    for _ in range(100):  # the angle stops at the vehicle's limit
        a.command([p.delta_rate_max, 0.0], 15.0)
    assert a.delta == pytest.approx(p.delta_max)


def lap(controller, dynamics, tmp_path, *extra):
    from carla_bridge import run

    carla.calls["generate_opendrive_world"].clear()
    args = ["--controller", controller, "--dynamics", dynamics, "--out", str(tmp_path), "--perfect"] + list(extra)
    status = run.main(args)
    with open(tmp_path / ("%s_%s.json" % (controller, dynamics))) as f:
        result = __import__("json").load(f)
    return status, result


@pytest.mark.slow
def test_pure_pursuit_laps_on_carla_dynamics(tmp_path):
    status, r = lap("pure_pursuit", "carla", tmp_path, "--no-video")
    assert status == 0 and r["completed"] and not r["collided"]
    assert r["road_margin"] > 0.0  # never on the grass
    assert len(carla.calls["generate_opendrive_world"]) == 1
    assert carla.calls["settings"][-2]["synchronous_mode"]  # ran synchronously, then put the settings back
    assert not carla.calls["settings"][-1]["synchronous_mode"]


@pytest.mark.slow
def test_nmpc_laps_on_carla_dynamics(tmp_path):
    status, r = lap("nmpc", "carla", tmp_path, "--no-video")
    assert status == 0 and r["completed"] and not r["collided"]
    assert r["road_margin"] > 0.0
    assert carla.calls["draw_line"] > 0  # the plan was drawn


@pytest.mark.slow
@pytest.mark.skipif(not trackctl.have_chrono, reason="built without Project Chrono")
def test_pure_pursuit_laps_on_chrono_dynamics(tmp_path):
    status, r = lap("pure_pursuit", "chrono", tmp_path, "--no-video")
    assert status == 0 and r["completed"] and not r["collided"]


@pytest.mark.slow
def test_nmpc_laps_on_bicycle_dynamics(tmp_path):
    status, r = lap("nmpc", "bicycle", tmp_path, "--no-video")
    assert status == 0 and r["completed"] and not r["collided"]


def video_size(path):
    import imageio_ffmpeg

    reader = imageio_ffmpeg.read_frames(str(path))
    size = next(reader)["size"]
    reader.close()
    return size


def test_rate_panel_widens_the_video(tmp_path):
    from carla_bridge.view import RatePanel

    lap("pure_pursuit", "carla", tmp_path, "--t-max", "0.5")
    assert video_size(tmp_path / "pure_pursuit_carla.mp4") == (1280 + RatePanel.WIDTH, 720)
    lap("pure_pursuit", "carla", tmp_path, "--t-max", "0.5", "--no-rates")
    assert video_size(tmp_path / "pure_pursuit_carla.mp4") == (1280, 720)


def test_rate_panel_averages():
    from carla_bridge.view import RatePanel

    panel = RatePanel(0.05)
    for i in range(40):
        panel.add(0.05 * (i + 1), 0.1, 0.02)  # the controller at 10 Hz, the plant at 50 Hz
    assert panel.average("controller") == pytest.approx(10.0)
    assert panel.average("plant") == pytest.approx(50.0)
    assert panel.draw(np.zeros((720, 1280, 3), np.uint8)).shape == (720, 1280 + RatePanel.WIDTH, 3)


def test_video_is_written(tmp_path):
    status, r = lap("pure_pursuit", "carla", tmp_path, "--t-max", "1.0")
    assert not r["completed"]  # stopped by the time limit
    video = tmp_path / "pure_pursuit_carla.mp4"
    assert video.exists() and video.stat().st_size > 0
    assert carla.calls["textured"] and carla.calls["weather"]  # the scenery was built


def _covered(planes, x, y):
    """For each point (x[i], y[i]), whether one of the flat planes covers it."""
    out = np.zeros(len(x), dtype=bool)
    for px, py, _, psi, scale, vertical in planes:
        assert not vertical
        dx, dy = x - px, y - py
        along = dx * math.cos(psi) + dy * math.sin(psi)
        across = -dx * math.sin(psi) + dy * math.cos(psi)
        out |= (np.abs(along) <= 5.0 * scale) & (np.abs(across) <= 6.0 * scale)
    return out


def test_scenery_draws_the_road():
    from carla_bridge import scenery

    track = CIRCUIT.track
    plan = scenery.layout(CIRCUIT)
    half = CIRCUIT.lane_width
    tol = scenery.TOLERANCE

    def points(e):
        poses = [track.pose(s, e) for s in np.arange(0.0, track.length, 0.5)]
        return np.array([p.x for p in poses]), np.array([p.y for p in poses])

    # asphalt everywhere on the road and the white layer under its edges, but for the scallops of up to
    # tol that the rows of planes leave on the inside of a corner
    for e in np.linspace(-half + tol, half - tol, 9):
        assert _covered(plan.planes["asphalt"], *points(e)).all(), e
    for e in (-half - scenery.WHITE_LINE + tol, half + scenery.WHITE_LINE - tol):
        assert _covered(plan.planes["white"], *points(e)).all(), e
    # the asphalt stays within the tolerance of its edge, so most of each edge line shows
    for e in (-half - tol - 0.01, half + tol + 0.01):
        assert not _covered(plan.planes["asphalt"], *points(e)).any(), e
    for e in (-half - scenery.WHITE_LINE - tol - 0.01, half + scenery.WHITE_LINE + tol + 0.01):
        assert not _covered(plan.planes["white"], *points(e)).any(), e
    # kerbs on the corners only, both sides
    kerbs = plan.planes["kerb_red"] + plan.planes["kerb_white"]
    for x, y, *_ in kerbs:
        s, e = track.project(x, y)
        assert track.curvature(s) != 0.0 and half < abs(e) < CIRCUIT.lane_width + CIRCUIT.grass_width
    # nothing to drive into: barriers outside the walls, trees and the crowd well beyond them
    e_wall = CIRCUIT.lane_width + CIRCUIT.grass_width
    for path, x, y, *_ in plan.meshes:
        clear = e_wall if path == scenery.GUARDRAIL else e_wall + 2.0
        assert abs(track.project(x, y)[1]) >= clear - 1e-6, path
    for x, y, *_ in plan.walkers:
        assert abs(track.project(x, y)[1]) > e_wall + 5.0


def test_scenery_textures_tile():
    from carla_bridge import scenery

    for name, image in scenery.textures().items():
        assert image.dtype == np.uint8 and image.ndim == 3 and image.shape[2] == 3, name
        # the grass and asphalt repeat over their planes: their opposite edges must match
        if image.shape[0] > 4:
            edge = np.abs(image[0].astype(int) - image[-1].astype(int)).mean()
            inner = np.abs(image[0].astype(int) - image[1].astype(int)).mean()
            assert edge < 3.0 * inner + 1.0, name


def test_live_stream_serves_frames():
    import socket
    import threading
    import urllib.request

    from carla_bridge.view import Stream

    with socket.socket() as s:  # a free port
        s.bind(("localhost", 0))
        port = s.getsockname()[1]
    stream = Stream(port)
    try:
        assert b"/stream" in urllib.request.urlopen("http://localhost:%d/" % port, timeout=5).read()
        frame = np.zeros((72, 128, 3), dtype=np.uint8)
        threading.Timer(0.3, stream.add, [frame]).start()  # a frame arrives while the client waits
        with urllib.request.urlopen("http://localhost:%d/stream" % port, timeout=5) as response:
            head = response.read(200)
        assert b"--frame" in head and b"image/jpeg" in head
    finally:
        stream.close()
