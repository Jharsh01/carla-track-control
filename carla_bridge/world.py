"""The CARLA side: the race-track world, the cars in it, and who moves the ego car.

Two vehicle dynamics drive the ego car:

- CarlaDynamics: CARLA's own (PhysX) vehicle. The controller's input becomes
  throttle, brake and steer (actuation.py) and the state is read back from the
  actor.
- ChronoDynamics: the Project Chrono sedan, stepped in this process
  (trackctl.ChronoPlant), with CARLA only drawing it: the ego actor's physics
  is off and its pose is written every tick, including Chrono's roll and pitch.
  The surface friction follows the track (asphalt or grass).

The traffic cars have no dynamics in the scenario: their actors have physics
off and are placed every tick where the scenario says they are.

The world runs in synchronous mode at a fixed step, so CARLA waits for the
controllers; an NMPC solve that takes longer than a step slows the simulation,
it does not change it.
"""
import math

import carla

from . import frames
from . import opendrive

EGO_BLUEPRINT = "vehicle.tesla.model3"  # what config/vehicles/model3.json describes
TRAFFIC_BLUEPRINTS = ["vehicle.audi.a2", "vehicle.nissan.micra", "vehicle.mini.cooper_s", "vehicle.citroen.c3",
                      "vehicle.seat.leon", "vehicle.toyota.prius"]


def connect(host, port, timeout=60.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def build_world(client, circuit, dt):
    """Load the track as an OpenDRIVE world, synchronous with step `dt`. Returns (world, original settings)."""
    xodr = opendrive.build(circuit)
    params = carla.OpendriveGenerationParameters(
        vertex_distance=1.0,  # mesh resolution along the road [m]; the 15 m corners need it finer than the default 2
        max_road_length=50.0,
        wall_height=1.0,  # a barrier at the outer edge of the grass
        additional_width=0.6,
        smooth_junctions=True,
        enable_mesh_visibility=True,
        enable_pedestrian_navigation=False)
    world = client.generate_opendrive_world(xodr, params)
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    settings.substepping = True  # PhysX steps of at most 10 ms inside each world step
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = max(1, int(math.ceil(dt / 0.01)))
    world.apply_settings(settings)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    world.tick()
    return world, original


def ground_offset(actor):
    """Height of an actor's origin above the ground it stands on: its bounding box's bottom is then at z = 0."""
    box = actor.bounding_box
    return box.extent.z - box.location.z


def transform(x, y, psi, z=0.0, roll=0.0, pitch=0.0):
    """CARLA transform of a pose in the library's frame (roll, pitch in rad: right side down, nose down)."""
    cx, cy = frames.to_carla_xy(x, y)
    return carla.Transform(carla.Location(x=cx, y=cy, z=z),
                           carla.Rotation(pitch=-math.degrees(pitch), yaw=frames.to_carla_yaw(psi),
                                          roll=math.degrees(roll)))


def spawn(world, blueprint_id, x, y, psi, physics, role):
    library = world.get_blueprint_library()
    blueprint = library.find(blueprint_id)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", role)
    if blueprint.has_attribute("color") and role == "hero":
        blueprint.set_attribute("color", "200,30,30")
    actor = world.spawn_actor(blueprint, transform(x, y, psi, z=0.6))
    actor.set_simulate_physics(physics)
    return actor


class Traffic:
    """The scenario's traffic cars, placed by the scenario every tick."""

    def __init__(self, world, circuit):
        self.circuit = circuit
        self.actors = []
        for i in range(circuit.n_traffic):
            pose = circuit.traffic_pose(i, 0.0)
            actor = spawn(world, TRAFFIC_BLUEPRINTS[i % len(TRAFFIC_BLUEPRINTS)], pose.x, pose.y, pose.psi,
                          physics=False, role="traffic_%d" % i)
            self.actors.append(actor)
        world.tick()
        self.z = [ground_offset(a) for a in self.actors]

    def place(self, t):
        for i, actor in enumerate(self.actors):
            pose = self.circuit.traffic_pose(i, t)
            actor.set_transform(transform(pose.x, pose.y, pose.psi, z=self.z[i]))

    def destroy(self):
        for actor in self.actors:
            actor.destroy()
        self.actors = []


class CarlaDynamics:
    """The ego car on CARLA's own vehicle physics."""

    name = "carla"

    def __init__(self, world, circuit, params, dt):
        from .actuation import Actuation

        self.world, self.circuit, self.params, self.dt = world, circuit, params, dt
        x0 = circuit.initial_state()
        self.actor = spawn(world, EGO_BLUEPRINT, *self._origin(x0), physics=True, role="hero")
        control = self.actor.get_physics_control()
        # the same steering angle for the same steer input at every speed
        control.steering_curve = [carla.Vector2D(x=0.0, y=1.0), carla.Vector2D(x=300.0, y=1.0)]
        self.actor.apply_physics_control(control)
        self.cg_ahead = control.center_of_mass.x  # the centre of mass is ahead of the actor origin [m]
        max_steer = control.wheels[0].max_steer_angle  # [deg]
        self.actuation = Actuation(params, max_steer, dt)
        world.tick()
        self.reset(x0)

    def _origin(self, x):
        X0, Y0 = frames.origin_from_cg(x[0], x[1], x[2], self.circuit.body.cg_to_center)
        return X0, Y0, x[2]

    def reset(self, x0):
        """Place the car at x0, rolling at its speed."""
        X0, Y0 = frames.origin_from_cg(x0[0], x0[1], x0[2], self.cg_ahead)
        self.actor.set_transform(transform(X0, Y0, x0[2], z=ground_offset(self.actor) + 0.05))
        self.actor.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0))
        self.world.tick()
        vx, vy = frames.velocity_to_carla(x0[2], x0[3])
        self.actor.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))
        self.actuation.reset(x0[6])

    def state(self):
        t = self.actor.get_transform()
        v = self.actor.get_velocity()
        w = self.actor.get_angular_velocity()
        return frames.state_from_carla((t.location.x, t.location.y), t.rotation.yaw, (v.x, v.y), w.z,
                                       self.cg_ahead, self.actuation.delta)

    def apply(self, u, x, friction):
        """Hold the input u for the next world step."""
        throttle, brake, steer = self.actuation.command(u, x[3])
        self.actor.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
        self.last_control = (throttle, brake, steer)

    def after_tick(self):
        pass

    def destroy(self):
        self.actor.destroy()


class ChronoDynamics:
    """The ego car on the Project Chrono sedan, drawn in CARLA."""

    name = "chrono"

    def __init__(self, world, circuit, params, dt):
        import trackctl

        if not trackctl.have_chrono:
            raise RuntimeError("trackctl was built without Project Chrono: run scripts/install_chrono.sh, then setup.sh")
        self.world, self.circuit, self.params, self.dt = world, circuit, params, dt
        self.plant = trackctl.ChronoPlant(params)
        x0 = circuit.initial_state()
        self.actor = spawn(world, EGO_BLUEPRINT, *self._origin(x0), physics=False, role="hero")
        world.tick()
        self.z = ground_offset(self.actor)
        self.reset(x0)

    def _origin(self, x):
        X0, Y0 = frames.origin_from_cg(x[0], x[1], x[2], self.circuit.body.cg_to_center)
        return X0, Y0, x[2]

    def reset(self, x0):
        self.plant.reset(x0)
        self.after_tick()

    def state(self):
        return self.plant.observe()

    def apply(self, u, x, friction):
        """Step Chrono through the next world step, the friction following the surface every 10 ms."""
        import numpy as np

        u = self.plant.saturate(np.asarray(u, dtype=float))
        steps = max(1, int(round(self.dt / 0.01)))
        for _ in range(steps):
            x = self.plant.observe()
            mu = friction(x)
            if math.isnan(mu):  # beyond the grass: the run ends on this
                break
            self.plant.set_friction(mu)
            self.plant.step(u, self.dt / steps)
        self.last_control = (float(u[1]), 0.0, 0.0)
        self._pose()

    def _pose(self):
        x = self.plant.observe()
        a = self.plant.attitude()
        X0, Y0, psi = self._origin(x)
        self.actor.set_transform(transform(X0, Y0, psi, z=self.z + a.heave, roll=a.roll, pitch=a.pitch))

    def after_tick(self):
        self._pose()

    def destroy(self):
        self.actor.destroy()


DYNAMICS = {"carla": CarlaDynamics, "chrono": ChronoDynamics}
