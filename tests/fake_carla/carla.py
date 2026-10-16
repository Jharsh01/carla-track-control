"""A stand-in for the `carla` client module, for testing the bridge without a CARLA server.

It implements the part of CARLA's Python API the bridge uses, with CARLA's
conventions: left-handed frame (y right, yaw clockwise, degrees), actor
locations at the actor origin, angular velocity in deg/s, steer positive to
the right. A vehicle with physics on is simulated by the library's bicycle
plant with the Model 3 parameters, driven through an "engine" that delivers
only ENGINE_EFFICIENCY of the force the throttle asks for, and a steering
servo, so that the bridge's frame conversions, actuation and feedback are
exercised end to end. Cameras deliver black images.

Static props, pedestrians and textures (the scenery) are only recorded.

Not CARLA: no rendering, no collisions, no tire model of CARLA's own.
"""
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "build", "python"))
import trackctl  # noqa: E402

ENGINE_EFFICIENCY = 0.85
CG_AHEAD = 0.45  # Model 3: centre of mass ahead of the actor origin [m]
MAX_STEER_DEG = 70.0
calls = {"generate_opendrive_world": [], "draw_line": 0, "settings": [], "textured": {}, "weather": []}


class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Location(Vector3D):
    pass


class Vector2D:
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = x, y


class Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or Location()
        self.rotation = rotation or Rotation()

    def transform(self, point):
        c, s = math.cos(math.radians(self.rotation.yaw)), math.sin(math.radians(self.rotation.yaw))
        return Location(self.location.x + c * point.x - s * point.y, self.location.y + s * point.x + c * point.y,
                        self.location.z + point.z)


class Color:
    def __init__(self, r=0, g=0, b=0, a=255):
        self.r, self.g, self.b, self.a = r, g, b, a


class VehicleControl:
    def __init__(self, throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False):
        self.throttle, self.steer, self.brake = throttle, steer, brake


class WeatherParameters:
    ClearNoon = "ClearNoon"

    def __init__(self, **values):
        self.__dict__.update(values)


class FloatColor(Color):
    pass


class TextureColor:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.pixels = {}

    def set(self, x, y, color):
        assert 0 <= x < self.width and 0 <= y < self.height
        self.pixels[x, y] = color


class TextureFloatColor(TextureColor):
    pass


class MaterialParameter:
    Normal, AO_Roughness_Metallic_Emissive, Diffuse, Emissive = range(4)


class command:
    class SpawnActor:
        def __init__(self, blueprint, transform, parent=None):
            self.blueprint, self.transform = blueprint, transform

    class SetSimulatePhysics:
        def __init__(self, actor, enabled):
            self.actor, self.enabled = actor, enabled


class Response:
    def __init__(self, actor_id=0, error=""):
        self.actor_id, self.error = actor_id, error


class AttachmentType:
    Rigid, SpringArm, SpringArmGhost = range(3)


class OpendriveGenerationParameters:
    def __init__(self, vertex_distance=2.0, max_road_length=50.0, wall_height=1.0, additional_width=0.6,
                 smooth_junctions=True, enable_mesh_visibility=True, enable_pedestrian_navigation=True):
        self.wall_height = wall_height


class WorldSettings:
    def __init__(self):
        self.synchronous_mode = False
        self.fixed_delta_seconds = None
        self.substepping = True
        self.max_substep_delta_time = 0.01
        self.max_substeps = 10


class BoundingBox:
    def __init__(self):
        self.extent = Vector3D(2.396, 1.082, 0.744)
        self.location = Vector3D(0.029, 0.0, 0.736)


class Wheel:
    def __init__(self, max_steer_angle):
        self.max_steer_angle = max_steer_angle


class PhysicsControl:
    def __init__(self):
        self.center_of_mass = Vector3D(CG_AHEAD, 0.0, -0.3)
        self.wheels = [Wheel(MAX_STEER_DEG), Wheel(MAX_STEER_DEG), Wheel(0.0), Wheel(0.0)]
        self.steering_curve = [Vector2D(0, 1.0), Vector2D(20, 0.9), Vector2D(60, 0.8), Vector2D(120, 0.7)]


class Blueprint:
    def __init__(self, id):
        self.id = id
        self.attributes = {"role_name": "", "color": ""} if id.startswith("vehicle") else {}

    def has_attribute(self, name):
        return name in self.attributes

    def set_attribute(self, name, value):
        self.attributes[name] = value


class BlueprintLibrary:
    def find(self, id):
        return Blueprint(id)


class Actor:
    def __init__(self, world, blueprint, transform):
        self.world = world
        self.type_id = blueprint.id
        self.attributes = dict(blueprint.attributes)
        self._transform = transform
        self.bounding_box = BoundingBox()
        self.alive = True

    def get_transform(self):
        return self._transform

    def set_transform(self, transform):
        self._transform = transform

    def destroy(self):
        self.alive = False
        self.world.actors.remove(self)
        return True


class Vehicle(Actor):
    def __init__(self, world, blueprint, transform):
        super().__init__(world, blueprint, transform)
        self.physics = True
        self.control = VehicleControl()
        self.physics_control = PhysicsControl()
        self.params = trackctl.VehicleParams.from_file(os.path.join(REPO, "config", "vehicles", "model3.json"))
        self.plant = trackctl.BicyclePlant(self.params)
        self._sync_plant(speed=0.0)

    def _sync_plant(self, speed):
        t = self._transform
        psi = -math.radians(t.rotation.yaw)
        x, y = t.location.x, -t.location.y
        x0 = np.zeros(7)
        x0[:4] = [x + CG_AHEAD * math.cos(psi), y + CG_AHEAD * math.sin(psi), psi, speed]
        self.plant.reset(x0)

    def set_simulate_physics(self, enabled):
        self.physics = enabled

    def set_transform(self, transform):
        self._transform = transform
        if self.physics:
            self._sync_plant(speed=self.plant.observe()[3])

    def get_physics_control(self):
        return self.physics_control

    def apply_physics_control(self, control):
        self.physics_control = control

    def apply_control(self, control):
        self.control = control

    def set_target_velocity(self, v):
        self._sync_plant(speed=math.hypot(v.x, v.y))

    def step(self, dt):
        if not self.physics:
            return
        x = self.plant.observe()
        Ux = max(x[3], 1.0)
        p = self.params
        # CARLA's steer: a fraction of the max angle (the curve is flat: the bridge set it), positive to the right
        curve = self.physics_control.steering_curve
        scale = curve[0].y if all(abs(c.y - curve[0].y) < 1e-9 for c in curve) else 0.8
        target = -self.control.steer * math.radians(self.physics_control.wheels[0].max_steer_angle) * scale
        rate = (target - x[6]) / 0.05
        drive = ENGINE_EFFICIENCY * self.control.throttle * min(p.F_drive_max, p.P_max / Ux)
        Fx = drive - self.control.brake * p.F_brake_max
        self.plant.step(np.array([rate, Fx]), dt)

    def _carla_state(self):
        x = self.plant.observe()
        X, Y, psi, Ux, Uy, r = x[:6]
        c, s = math.cos(psi), math.sin(psi)
        Uy_origin = Uy - r * CG_AHEAD
        vx, vy = c * Ux - s * Uy_origin, s * Ux + c * Uy_origin
        return (X - CG_AHEAD * c, -(Y - CG_AHEAD * s), -math.degrees(psi), vx, -vy, -math.degrees(r))

    def get_transform(self):
        if not self.physics:
            return self._transform
        ox, oy, yaw, _, _, _ = self._carla_state()
        return Transform(Location(ox, oy, self._transform.location.z), Rotation(yaw=yaw))

    def get_velocity(self):
        if not self.physics:
            return Vector3D()
        _, _, _, vx, vy, _ = self._carla_state()
        return Vector3D(vx, vy, 0.0)

    def get_angular_velocity(self):
        if not self.physics:
            return Vector3D()
        return Vector3D(0.0, 0.0, self._carla_state()[5])


class Image:
    def __init__(self, frame, width, height):
        self.frame, self.width, self.height = frame, width, height
        self.raw_data = bytes(width * height * 4)


class Camera(Actor):
    def __init__(self, world, blueprint, transform, parent):
        super().__init__(world, blueprint, transform)
        self.parent = parent
        self.size = (int(blueprint.attributes.get("image_size_x", 800)), int(blueprint.attributes.get("image_size_y", 600)))
        self.callback = None

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.callback = None


class CameraBlueprint(Blueprint):
    def __init__(self, id):
        super().__init__(id)
        self.attributes = {}

    def has_attribute(self, name):
        return True


class Debug:
    def draw_line(self, a, b, thickness=0.1, color=None, life_time=-1.0):
        calls["draw_line"] += 1


class World:
    def __init__(self):
        self.settings = WorldSettings()
        self.actors = []
        self.frame = 0
        self.debug = Debug()
        self.spectator = Actor(self, Blueprint("spectator"), Transform())
        self.props = []  # static meshes and pedestrians: never moved, not stepped

    def get_settings(self):
        s = WorldSettings()
        s.__dict__.update(self.settings.__dict__)
        return s

    def apply_settings(self, settings):
        calls["settings"].append(dict(settings.__dict__))
        self.settings = settings
        return self.frame

    def set_weather(self, weather):
        calls["weather"].append(weather)

    def get_blueprint_library(self):
        class Library:
            def find(self, id):
                return CameraBlueprint(id) if id.startswith("sensor") else Blueprint(id)

            def filter(self, pattern):
                return [Blueprint(pattern.replace("*", "%04d" % i)) for i in range(1, 4)]

        return Library()

    def get_names_of_all_objects(self):
        return ["BP_Sky"] + ["StaticMeshActor_%d" % i for i in range(len(self.props))]

    def apply_color_texture_to_objects(self, names, parameter, texture):
        for name in names:
            calls["textured"][name] = texture

    def apply_float_color_texture_to_objects(self, names, parameter, texture):
        pass

    def get_spectator(self):
        return self.spectator

    def spawn_actor(self, blueprint, transform, attach_to=None, attachment_type=None):
        if blueprint.id.startswith("sensor"):
            actor = Camera(self, blueprint, transform, attach_to)
        else:
            actor = Vehicle(self, blueprint, transform)
        self.actors.append(actor)
        return actor

    def tick(self, seconds=10.0):
        dt = self.settings.fixed_delta_seconds or 0.05
        for actor in list(self.actors):
            if isinstance(actor, Vehicle):
                actor.step(dt)
        self.frame += 1
        for actor in list(self.actors):
            if isinstance(actor, Camera) and actor.callback:
                actor.callback(Image(self.frame, *actor.size))
        return self.frame


class Client:
    def __init__(self, host="localhost", port=2000, worker_threads=0):
        self.world = None

    def set_timeout(self, seconds):
        pass

    def get_server_version(self):
        return "fake"

    def get_client_version(self):
        return "fake"

    def generate_opendrive_world(self, opendrive, parameters=None, reset_settings=True):
        calls["generate_opendrive_world"].append(opendrive)
        self.world = World()
        return self.world

    def get_world(self):
        return self.world

    def apply_batch_sync(self, commands, do_tick=False):
        responses = []
        for c in commands:
            if isinstance(c, command.SpawnActor):
                assert c.blueprint.id.startswith(("static.", "walker.")), c.blueprint.id
                self.world.props.append(Actor(self.world, c.blueprint, c.transform))
                responses.append(Response(actor_id=len(self.world.props)))
            else:
                responses.append(Response(actor_id=c.actor))
        if do_tick:
            self.world.tick()
        return responses
