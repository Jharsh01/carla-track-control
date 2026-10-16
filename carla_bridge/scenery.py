"""What the race track looks like in CARLA: asphalt, kerbs, grass, barriers, trees and a grandstand.

CARLA draws a world generated from OpenDRIVE with one untextured material,
road and walls alike, over an empty void. The bridge therefore hides that mesh
(it keeps its collision, so CARLA's car still drives on it and the walls still
stop it) and draws the track itself with what a packaged CARLA can spawn at run
time:

- the ground, the asphalt, the white edge lines, the kerbs and the start line
  are flat planes (CARLA's SM_Plane) given a texture made here
  (World.apply_color_texture_to_objects). A plane's texture repeats 10 x 12
  times over it and CARLA scales a mesh uniformly only, so a plane is
  10 x 12 tiles of `scale` metres each. They lie a centimetre or so under the
  road's collision surface, in layers: grass, kerbs, white, asphalt, start line;
  the white layer is wider than the asphalt, and what shows of it is the edge
  lines.
- On a corner, planes are short enough that the polygon they make follows the
  arc within a few centimetres; on a straight, one plane spans the road.
- The barriers are Armco guardrails along the walls; the trees and the
  grandstand are CARLA's own meshes, the sponsor boards are planes standing up
  and the crowd is CARLA's pedestrians, their physics off once they stand on
  the steps.
- Meshes spawned this way cannot be moved afterwards: everything is placed once,
  where it stays.

layout() computes everything from the circuit alone, in the library's frame
(x forward, y left, radians), so it can be checked without CARLA; build() puts
it in a CARLA world.
"""
import math
import random

import numpy as np

PLANE = "/Game/Carla/Static/Other/SM_Plane.SM_Plane"
PLANE_SURFACE = 0.1  # SM_Plane's surface is this far (x scale) from its origin, on its textured side [m]
GUARDRAIL = "/Game/Carla/Static/GuardRail/SM_Guardrail_02.SM_Guardrail_02"
GUARDRAIL_LENGTH = 3.11  # [m], from its origin along +x
GRANDSTAND = "/Game/Carla/Static/Other/SM_StairField.SM_StairField"
TREES = ["Oak_L_v1", "Oak_L_v3", "Oak_M_v3", "Oak_M_v5", "Pine_L_v3", "Pine_L_v6", "Pine_M_v5", "Beech", "Ash_01",
         "Ash_02", "Sassafras_01", "Sassafras_03", "Maple_M_v3", "Cypress", "TreePine_2", "Acer_02", "Wallnut_01"]

# heights of the layers [m]: under the road's (invisible) collision surface at 0, so the car rests on that
Z_GROUND, Z_KERB, Z_WHITE, Z_ASPHALT, Z_LINE = -0.045, -0.034, -0.024, -0.014, -0.006
WHITE_LINE = 0.15  # edge line width [m]
KERB_WIDTH = 0.9  # what shows of a kerb outside the edge line [m]
KERB_STRIPE = 1.0  # length of a red or white block [m]
TOLERANCE = 0.04  # how far the planes' outline may stray from the road's edge on a corner [m]


class Layout:
    def __init__(self):
        self.planes = {}  # texture name -> [(x, y, z, psi, scale, vertical)]
        self.meshes = []  # (mesh path, x, y, z, psi, scale)
        self.walkers = []  # spectators: (x, y, z, psi)
        self.bounds = None  # (x_min, y_min, x_max, y_max) of the track

    def plane(self, texture, x, y, z, psi, scale, vertical=False):
        self.planes.setdefault(texture, []).append((x, y, z, psi, scale, vertical))

    @property
    def count(self):
        return sum(len(p) for p in self.planes.values()) + len(self.meshes) + len(self.walkers)


def _max_curvature(track, s, half_window):
    return max(abs(track.curvature((s + d) % track.length)) for d in np.linspace(-half_window, half_window, 9))


def band(layout, track, texture, e_lo, e_hi, z, tol=TOLERANCE):
    """Cover the strip e_lo <= e_y <= e_hi along the whole track with planes."""
    width = e_hi - e_lo
    full = 10.0 * width / 12.0  # length of one plane spanning the strip
    e_far = max(abs(e_lo), abs(e_hi))

    def planes_at(s):
        """(planes across, their scale, the longest step to the next row) for a row centred at s."""
        kappa = _max_curvature(track, s, full)
        if kappa < 1e-9:
            return 1, width / 12.0, 0.95 * full
        R = 1.0 / kappa
        # a plane is a chord of the arc: its corners stand out by L^2 / (8 R) on the outside
        n = max(1, int(math.ceil(full / math.sqrt(8.0 * tol * (R + e_far)))))
        L = full / n
        # consecutive rows must still touch at the outer edge, and leave scallops of at most tol on the inner one
        return n, width / (12.0 * n), min(0.95 * L * R / (R + e_far), math.sqrt(8.0 * tol * max(R - e_far, 1.0)))

    s, i = 0.0, 0
    first_half = 5.0 * planes_at(0.0)[1]  # how far the first row reaches back from the start
    while True:
        n, scale, step = planes_at(s % track.length)
        if s - 5.0 * scale >= track.length - first_half:  # the lap is closed: this row would only overlap
            break
        # the next row may be shorter (a corner begins): the two rows must overlap
        n_next, scale_next, step_next = planes_at((s + step) % track.length)
        step = min(step, step_next, 0.95 * 5.0 * (scale + scale_next))
        for k in range(n):
            e = e_lo + (k + 0.5) * width / n
            p = track.pose(s % track.length, e)
            layout.plane(texture, p.x, p.y, z + 0.001 * (i % 2), p.psi, scale)
        s += step
        i += 1


def kerbs(layout, track, half_road):
    """Red and white blocks outside the edge lines on every corner, both sides."""
    width = KERB_WIDTH + 0.3  # the inner 0.3 m is under the white layer
    scale = width / 12.0
    # each block is a plane of length 10 * scale; enough of them, slightly overlapping, make a stripe
    for side in (1.0, -1.0):
        e = side * (half_road + WHITE_LINE + KERB_WIDTH - width / 2.0)
        s, colour, run = 0.0, 0, 0.0
        while s < track.length:
            kappa = track.curvature(s)
            if kappa == 0.0:
                s += 0.5
                run = 0.0
                continue
            # along the kerb's centre the track's s advances by (1 - kappa e) per metre
            stretch = 1.0 - kappa * e
            p = track.pose(s, e)
            texture = "kerb_red" if colour % 2 else "kerb_white"
            layout.plane(texture, p.x, p.y, Z_KERB + 0.001 * (colour % 2), p.psi, scale)
            s += (10.0 * scale * 0.97) / stretch
            run += 10.0 * scale * 0.97
            if run >= KERB_STRIPE - 1e-6:
                colour += 1
                run = 0.0


def start_line(layout, track, s0, half_road, squares=14, rows=2):
    """A chequered strip across the asphalt at s0."""
    size = 2.0 * half_road / squares
    scale = size / 12.0
    depth = 10.0 * scale
    for r in range(rows):
        for q in range(squares):
            e = -half_road + (q + 0.5) * size
            p = track.pose(s0 + (r - (rows - 1) / 2.0) * depth, e)
            layout.plane("chequer_black" if (q + r) % 2 else "chequer_white", p.x, p.y, Z_LINE, p.psi, scale)


def guardrails(layout, track, e_wall):
    """Armco along the line e_y = e_wall, as chords of GUARDRAIL_LENGTH; the rail stands outside the line."""
    pts = [track.pose(s, e_wall) for s in np.arange(0.0, track.length, 0.25)]
    pts.append(pts[0])
    a = pts[0]
    for b in pts[1:]:
        if math.hypot(b.x - a.x, b.y - a.y) < GUARDRAIL_LENGTH:
            continue
        psi = math.atan2(b.y - a.y, b.x - a.x)
        # the rail's body is on its left (in the library's frame) when it points from its origin along +x;
        # on the left wall that is outwards, on the right wall it is turned round
        if e_wall > 0:
            layout.meshes.append((GUARDRAIL, a.x, a.y, 0.0, psi, 1.0))
        else:
            layout.meshes.append((GUARDRAIL, b.x, b.y, 0.0, psi + math.pi, 1.0))
        a = b


def trees(layout, track, clear, rng, spacing=9.0, depth=45.0):
    """Woods both sides of the track, at least `clear` metres from its centreline."""
    for side in (1.0, -1.0):
        for s in np.arange(0.0, track.length, spacing):
            for _ in range(2):
                e = side * (clear + 2.0 + rng.random() * depth)
                p = track.pose(s + rng.uniform(-spacing, spacing) / 2.0, e)
                if abs(track.project(p.x, p.y)[1]) < clear:  # on the infield of a corner: too near another stretch
                    continue
                if rng.random() < 0.25:
                    continue
                name = rng.choice(TREES)
                layout.meshes.append(("/Game/Carla/Static/Vegetation/Trees/SM_%s.SM_%s" % (name, name),
                                      p.x, p.y, 0.0, rng.uniform(0, 2 * math.pi), rng.uniform(0.8, 1.3)))


def ground(layout, bounds, margin=160.0, scale=8.0, far_scale=250.0):
    """Grass under everything: a grid of planes, each 10 x 12 tiles of `scale` metres, and one plane
    reaching to the horizon under them."""
    x0, y0, x1, y1 = bounds
    lx, ly = 10.0 * scale, 12.0 * scale  # a plane with psi = 0 is 10 tiles along x
    for x in np.arange(x0 - margin, x1 + margin, lx * 0.999):
        for y in np.arange(y0 - margin, y1 + margin, ly * 0.999):
            layout.plane("grass", x + lx / 2.0, y + ly / 2.0, Z_GROUND, 0.0, scale)
    layout.plane("grass_far", (x0 + x1) / 2.0, (y0 + y1) / 2.0, Z_GROUND - 0.5, 0.0, far_scale)


def layout(circuit, seed=0):
    """Everything to draw for `circuit`, in the library's frame."""
    track = circuit.track
    half_road = circuit.lane_width  # the centreline is the lane divider: one lane each side
    e_wall = circuit.lane_width + circuit.grass_width
    rng = random.Random(seed)
    out = Layout()
    xs, ys = [], []
    for s in np.arange(0.0, track.length, 2.0):
        for e in (-e_wall, e_wall):
            p = track.pose(s, e)
            xs.append(p.x)
            ys.append(p.y)
    out.bounds = (min(xs), min(ys), max(xs), max(ys))

    ground(out, out.bounds)
    kerbs(out, track, half_road)
    band(out, track, "white", -half_road - WHITE_LINE, half_road + WHITE_LINE, Z_WHITE)
    band(out, track, "asphalt", -half_road, half_road, Z_ASPHALT)
    x0 = circuit.initial_state()
    s_start = track.project(x0[0], x0[1])[0]
    start_line(out, track, s_start, half_road)
    for side in (1.0, -1.0):
        guardrails(out, track, side * (e_wall + 0.02))
    grandstand(out, circuit, s_start, e_wall, rng)
    trees(out, track, clear=e_wall + 12.0, rng=rng)
    return out


def _outside(circuit):
    """+1 if the outside of the circuit is to the left of the direction of travel, else -1."""
    track = circuit.track
    turning = sum(k * l for k, l in zip(track.piece_curvatures, track.piece_lengths))
    return -1.0 if turning > 0 else 1.0  # counter-clockwise: the outside is on the right


def grandstand(out, circuit, s_start, e_wall, rng):
    """A grandstand and sponsor boards on the outside of the start straight, before and after the line."""
    track = circuit.track
    side = _outside(circuit)
    # the straight the line is on: as far as the curvature stays zero either way
    s_a = s_start
    while s_a > s_start - 400 and track.curvature(s_a - 1.0) == 0.0:
        s_a -= 1.0
    s_b = s_start
    while s_b < s_start + 400 and track.curvature(s_b + 1.0) == 0.0:
        s_b += 1.0
    # boards: vertical planes 3 m long, 1.1 m above the ground, just behind the rail, facing the track
    board_scale = 0.25
    colours = ["board_red", "board_white", "board_blue", "board_yellow", "board_black", "board_green"]
    for k, s in enumerate(np.arange(s_a + 4.0, s_b - 4.0, 12.0 * board_scale)):
        p = track.pose(s, side * (e_wall + 0.6))
        out.plane(colours[k % len(colours)], p.x, p.y, 1.1 - 5.0 * board_scale, p.psi, board_scale, vertical=side)
    # the stand: CARLA's terraced steps (20 m long along their local y), 3 x their size, along the
    # straight and facing the track, centred on the line
    for d in (-31.0, 31.0):
        p = track.pose(s_start + d, side * (e_wall + 9.0))
        out.meshes.append((GRANDSTAND, p.x, p.y, 0.0, p.psi - side * math.pi / 2.0, 3.0))
    # the crowd: dropped onto the steps from above, facing the track
    for s in np.arange(s_start - 60.0, s_start + 60.0, 1.6):
        for row in range(4):
            if rng.random() < 0.8:
                continue
            p = track.pose(s + rng.uniform(-0.5, 0.5), side * (e_wall + 7.9 + 0.75 * row))
            out.walkers.append((p.x, p.y, 3.5, p.psi - side * math.pi / 2.0 + rng.uniform(-0.3, 0.3)))


# --- textures -------------------------------------------------------------------------------------------------

def _noise(n, rng, smooth):
    """Periodic (tileable) noise in [-1, 1], low-passed: a feature size of about n / smooth pixels."""
    f = np.fft.fftfreq(n)
    k = np.sqrt(f[:, None] ** 2 + f[None, :] ** 2)
    spectrum = np.fft.fft2(rng.standard_normal((n, n))) * np.exp(-(k * smooth) ** 2)
    a = np.real(np.fft.ifft2(spectrum))
    return a / (np.abs(a).max() + 1e-12)


def textures(seed=0):
    """name -> RGB array (n, n, 3) uint8; the first axis runs along the track."""
    rng = np.random.default_rng(seed)
    out = {}
    # grass: a plane tile is 8 m; patches of a few metres, mown stripes, blades
    n = 512
    g = 0.55 * _noise(n, rng, 6.0) + 0.3 * _noise(n, rng, 40.0) + 0.25 * rng.standard_normal((n, n))
    stripes = 0.12 * np.sign(np.sin(np.arange(n) / n * 2 * np.pi * 2))[:, None]
    base = np.array([40.0, 72.0, 24.0])
    tint = np.array([0.3, 0.4, 0.2])
    out["grass"] = np.clip(base + 60.0 * (g + stripes)[..., None] * tint, 0, 255).astype(np.uint8)
    # far away: the same grass without the mown stripes, which would be 125 m wide there
    out["grass_far"] = np.clip(base + 60.0 * g[..., None] * tint, 0, 255).astype(np.uint8)
    # asphalt: dark, fine grain with lighter stones
    n = 64
    a = 0.5 * _noise(n, rng, 3.0) + 0.5 * rng.standard_normal((n, n))
    asphalt = 38.0 + 7.0 * a
    asphalt += 30.0 * (rng.random((n, n)) > 0.97)
    out["asphalt"] = np.clip(np.repeat(asphalt[..., None], 3, axis=2) * np.array([1.0, 1.0, 1.04]), 0, 255).astype(np.uint8)

    def flat(rgb):
        return np.tile(np.array(rgb, dtype=np.uint8), (4, 4, 1))

    out["white"] = flat((228, 228, 224))
    out["kerb_red"] = flat((196, 22, 22))
    out["kerb_white"] = flat((236, 236, 232))
    out["chequer_black"] = flat((18, 18, 18))
    out["chequer_white"] = flat((240, 240, 240))
    for name, rgb in [("red", (200, 30, 30)), ("white", (235, 235, 235)), ("blue", (25, 60, 170)),
                      ("yellow", (240, 200, 20)), ("black", (25, 25, 25)), ("green", (20, 130, 60))]:
        out["board_" + name] = flat(rgb)
    return out


# --- CARLA ----------------------------------------------------------------------------------------------------

def _texture(carla, rgb):
    n, m, _ = rgb.shape
    t = carla.TextureColor(n, m)
    for i in range(n):
        for j in range(m):
            r, g, b = rgb[i, j]
            t.set(i, j, carla.Color(int(r), int(g), int(b), 255))
    return t


def _carla_transform(carla, x, y, z, psi, vertical=False, pitch=0.0, scale=1.0):
    """A pose in the library's frame as a CARLA transform; a plane lies flat, its 10-tile side along psi.

    A flat plane's surface is then at height z (its mesh's surface is PLANE_SURFACE * scale off its origin).

    A vertical plane (a board) stands along psi, its textured face towards the track: `vertical` is the side
    (+1 left, -1 right) it is on.
    """
    from . import frames

    cx, cy = frames.to_carla_xy(x, y)
    if pitch == -90.0:
        z -= PLANE_SURFACE * scale
    if vertical:
        # the plane's normal is its local x; its textured face looks along -x: point x away from the track
        nx, ny = -math.sin(psi) * vertical, math.cos(psi) * vertical
        yaw = frames.to_carla_yaw(math.atan2(ny, nx))
        return carla.Transform(carla.Location(x=cx, y=cy, z=z), carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0))
    return carla.Transform(carla.Location(x=cx, y=cy, z=z),
                           carla.Rotation(pitch=pitch, yaw=frames.to_carla_yaw(psi), roll=0.0))


def build(client, world, circuit, seed=0):
    """Draw the circuit in `world` (synchronous mode). Returns the number of actors spawned."""
    import carla

    plan = layout(circuit, seed)
    library = world.get_blueprint_library()

    def mesh_blueprint(path, scale):
        bp = library.find("static.prop.mesh")
        bp.set_attribute("mesh_path", path)
        bp.set_attribute("scale", "%.6g" % scale)
        return bp

    def spawn(commands):
        """Spawn a batch; returns the names of the scene objects it added, for retexturing."""
        before = set(world.get_names_of_all_objects())
        responses = client.apply_batch_sync(commands, False)
        n = sum(1 for r in responses if not r.error)
        for _ in range(20):
            world.tick()
            added = set(world.get_names_of_all_objects()) - before
            if len(added) >= n:
                break
        return sorted(added), n

    images = textures(seed)
    matte = carla.TextureFloatColor(2, 2)
    for i in range(2):
        for j in range(2):
            matte.set(i, j, carla.FloatColor(1.0, 1.0, 0.0, 0.0))
    spawned = 0
    for name, planes in plan.planes.items():
        commands = []
        for x, y, z, psi, scale, vertical in planes:
            t = _carla_transform(carla, x, y, z, psi, vertical, pitch=-90.0, scale=scale)
            commands.append(carla.command.SpawnActor(mesh_blueprint(PLANE, scale), t))
        names, n = spawn(commands)
        spawned += n
        if names:
            world.apply_color_texture_to_objects(names, carla.MaterialParameter.Diffuse, _texture(carla, images[name]))
            # matte: ambient occlusion 1, roughness 1, not metallic, not emissive
            world.apply_float_color_texture_to_objects(names, carla.MaterialParameter.AO_Roughness_Metallic_Emissive,
                                                       matte)
    commands = [carla.command.SpawnActor(mesh_blueprint(path, scale), _carla_transform(carla, x, y, z, psi))
                for path, x, y, z, psi, scale in plan.meshes]
    spawned += spawn(commands)[1]
    people = library.filter("walker.pedestrian.*")
    rng = random.Random(seed)
    commands = [carla.command.SpawnActor(rng.choice(people), _carla_transform(carla, x, y, z, psi))
                for x, y, z, psi in plan.walkers]
    crowd = [r.actor_id for r in client.apply_batch_sync(commands, False) if not r.error]
    spawned += len(crowd)
    for _ in range(40):  # the crowd lands on the steps
        world.tick()
    # then stands still: walker physics costs more per tick than everything else here
    client.apply_batch_sync([carla.command.SetSimulatePhysics(a, False) for a in crowd], False)
    world.set_weather(carla.WeatherParameters(cloudiness=25.0, precipitation=0.0, precipitation_deposits=0.0,
                                              wind_intensity=10.0, sun_azimuth_angle=150.0,
                                              sun_altitude_angle=65.0, fog_density=0.0, wetness=0.0))
    world.tick()
    return spawned
