# Vehicle parameters: CARLA Tesla Model 3

The model's default parameters (`parameters.json`, loaded by `parameters.cpp`) describe
`vehicle.tesla.model3` in CARLA 0.9.15.

**Sources**

- The CARLA documentation gives the meaning and units of each physics field
  ([Python API reference](https://carla.readthedocs.io/en/0.9.15/python_api/),
  [vehicle physics tutorial](https://carla.readthedocs.io/en/0.9.15/tuto_G_control_vehicle_physics/))
  but lists no per-vehicle values; the
  [vehicle catalogue](https://carla.readthedocs.io/en/0.9.15/catalogue_vehicles/)
  entry for the Model 3 has no physical specifications.
- The values were read from the blueprint with `Vehicle.get_physics_control()`
  on a CARLA 0.9.15 server. The raw readout is in
  [`carla/model3_physics_control_0.9.15.json`](carla/model3_physics_control_0.9.15.json).

No driving tests have been run in CARLA. Everything marked *assumed* is an
estimate that a test still has to confirm.

## Read from CARLA

| Quantity | Value | Unit (per docs) |
| --- | --- | --- |
| Mass | 1845 | kg |
| Drag coefficient | 0.15 | – |
| Centre of mass offset | (0.45, 0, −0.30) | m |
| Wheel radius | 37 | cm |
| Max steer angle, front wheels | 70 | degrees |
| Steering curve (speed → scale) | 0 → 1.0, 20 → 0.9, 60 → 0.8, 120 → 0.7 | – |
| Max brake torque, each wheel | 700 | N·m |
| Tire friction, each wheel | 3.5 | – |
| Lateral stiffness value / max load | 20 / 3 | – |
| Longitudinal stiffness value | 3000 | – |
| Torque curve | 743 N·m to 9000 rpm, falling to 502 N·m at 15000 rpm | – |
| Final drive ratio, gear ratio | 9.0, 1.0 (single gear) | – |
| Bounding box (L × W × H) | 4.79 × 2.16 × 1.49 | m |
| Wheel positions | front axle 1.618 m ahead of the actor origin, rear axle 1.386 m behind, track 1.667 m | world frame, cm |

## Model parameters

| Parameter | Value | Basis |
| --- | --- | --- |
| `m` | 1845 kg | Read directly. |
| `L` (wheelbase) | 3.005 m | Derived: distance between front and rear wheel positions. |
| `a`, `b` | 1.168 m, 1.836 m | Derived, **with an assumption**: the centre-of-mass offset (0.45 m forward) is taken relative to the actor origin. The docs say only "center of mass of the vehicle". This gives 61 % of the weight on the front axle. |
| `h` | 0.45 m | **Assumed.** CARLA gives only a vertical offset (−0.30 m) without a reference height. |
| `Izz` | 3958 kg·m² | **Assumed:** `m·a·b`. CARLA does not expose the yaw inertia (`moi` is the engine's). |
| `C_alpha_f`, `C_alpha_r` | 106.5, 67.8 kN/rad | Derived, **with an assumption** about PhysX's tire model: stiffness per tire = rest load × `lat_stiff_value` × s(1 / `lat_stiff_max_load`), with s(x) = 1.5x − 0.5x³, i.e. 9.63 × static axle load. Because stiffness scales with load, the car is neutral-steer in the linear range (understeer gradient ≈ 0). |
| `mu` | 0.9 | **Assumed**, and a scenario variable. `tire_friction` = 3.5 is a PhysX multiplier combined with the road surface, not a usable friction coefficient. |
| `F_drive_max` | 18 073 N | Derived: 743 N·m × 9.0 / 0.37 m. Constant up to 9000 rpm = 38.7 m/s (139 km/h). |
| `P_max` | 700 kW | Derived: 743 N·m at 9000 rpm. Not limiting below 139 km/h. |
| `drive_front` | 0.5 | **Assumed** four-wheel drive with an even split; the differential type is not exposed. |
| `F_brake_max` | 7568 N | Derived: 4 × 700 N·m / 0.37 m. This is 4.10 m/s² (0.42 g): the brakes, not the tires, limit deceleration on a dry road. |
| `brake_front` | 0.5 | Derived: equal brake torque on all four wheels. |
| `CdA` | 0.483 m² | Derived, **with an assumption**: drag area = bounding-box width × height (3.22 m²). |
| `Crr` | 0.012 | **Assumed.** Not a CARLA parameter. |
| `delta_max` | 0.855 rad (49°) | Derived: 70° × 0.7, the steering-curve scale at 120 km/h. Larger at lower speed (assuming the curve's speed axis is km/h). |
| `delta_rate_max` | 0.6 rad/s | Design choice for the controller, not a CARLA property. |

## To confirm with tests in CARLA

| Assumption | Test |
| --- | --- |
| Peak braking is 0.42 g | Full-brake stop from 30 m/s on a straight. |
| Friction coefficient | Peak lateral acceleration in a ramp steer; peak deceleration with brake torque raised. |
| Cornering stiffnesses, neutral steer | Small step steer at 10, 20, 30 m/s: yaw-rate gain and axle slip angles. |
| Centre-of-mass position, yaw inertia | Step-steer transient fit. |
| Drag area, rolling resistance | Coast-down from 35 m/s. |
| Drive split and peak drive force | Full-throttle run from rest. |
| Steering scale against speed | `get_wheel_steer_angle` at several speeds. |
