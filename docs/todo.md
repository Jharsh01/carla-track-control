# To do

Open work on the NMPC with pure pursuit fallback (`--controller nmpc_fallback`), with what has been
measured so far. File references are to the code as of this commit.

## 1. Stop the NMPC oscillating on the straights (2 s horizon)

**Symptom.** With the fallback defaults (2 s horizon, 20 ms steps for the first 0.2 s, then
0.1 s), the NMPC drives bang-bang on the straights: Fx swings between about −3.5 kN and +4.5 kN
from one solve to the next, the steering follows (std about 2° in CARLA against 0.3–0.4° with the
4 s horizon), and the car hangs between the lanes (e_y −0.1 to −0.9 m instead of its lane centre at
−1.75 m). The speed measured at successive solves alternates, e.g. 20.02, 19.39, 19.83, 19.24,
19.78, 19.37 m/s, with no traffic and no constraint active.

**Cause.** The road is known `--road-range` (40 m) ahead. Past that the NMPC limits the planned
speed to `unseen_speed`, the speed of the tightest corner at the road's friction (about 11.5 m/s),
so that it can always brake for what comes into view. Which steps are past it is decided from the
previous plan shifted forward (`s_plan`, `track_control/src/nmpc.cpp`, where `v_lim` is set). At
about 20 m/s a 2 s horizon ends at almost exactly 40 m, so on alternate solves:

- the previous plan ended inside 40 m: no step is limited, so the plan accelerates;
- that plan, shifted, ends past 40 m: its last steps are limited to 11.5 m/s, so the plan brakes;
- the braking plan ends inside 40 m again, and so on.

A 4 s horizon reaches about 80 m, always past the limit, so every solve sees it the same way.

**Measured offline** (`build/simulate --controller nmpc --plant bicycle`, a solve every 50 ms, the
straights at s 633–853 m and 920–1160 m):

| horizon | road known | Fx std | Fx sign changes | lowest e_y reached |
|---|---|---|---|---|
| 4 s | 40 m | 650 N | about 95 | −1.75 m |
| 2 s | 40 m | 3950 N | about 230 | −0.4 to −0.6 m |
| 2 s, 20 ms steps for 0.2 s | 40 m | 3990 N | about 230 | −0.4 to −0.6 m |
| 2 s | 80 m | 1400 N | about 60 | −1.4 to −1.5 m |

The variable step does not change it; moving the road range away from the horizon's end mostly
removes it. In CARLA the 200 ms open-loop playback roughly triples the steering swing (2° against
0.6° offline).

### 1a. Back to a 4 s horizon, keeping the variable step

- Set the `nmpc_fallback` default `--nmpc-horizon` to 4 (`carla_bridge/run.py`, `parse`).
- Offline, 20 ms steps for 0.2 s then 0.1 s to 4 s (48 steps): solve p50 93 ms, p95 108 ms, max
  123 ms through the hairpin, inside the 200 ms period.
- Check: a CARLA lap or two, solves on time, the straights' Fx std back near 650 N.

### 1b. A smooth speed limit instead of the step at the end of the known road

- Replace the step from `road.v_max` to `unseen_speed` at `s0 + road_range` with the braking
  envelope over the known road:
  `v_lim(s) = min(v_max, sqrt(unseen_speed² + 2 a_brake max(0, s0 + road_range − s)))`,
  with `a_brake` from the planned brake force (`F_brake_max / m`, after `brake_use`).
- It is continuous in s, so the plan no longer depends on whether its end falls just inside or
  just outside the known road; a 2 s horizon should then be steady too.
- Check: the offline table above for 2 s and 4 s, the lap time and the asphalt margin against
  the current 72.2 s and 0.19 m (4 s horizon), `track_control.test_controllers`.

## 2. Make pure pursuit behave more like the NMPC

Pure pursuit takes over (blended over `--blend`) whenever the NMPC has no plan for the moment.
The first crash (pure pursuit braking at the full 7568 N mid-hairpin, 7° of sideslip, the car
spinning onto the grass) came from how differently the two drive. Ideas, the first two the most
useful:

1. **Follow the NMPC's last plan.** While the NMPC is late, steer along the last plan's (s, e_y)
   and track its speed instead of pure pursuit's own lane and speed profile; the plan covers the
   whole horizon, longer than any outage so far.
2. **Brake within the grip left.** Cap |Fx| by the friction circle,
   `sqrt((mu m g)² − (m Ux r)²)`, so it never brakes at the limit in a corner.
3. **The same speed targets.** Its profile uses 40% of the grip (`grip_use`), the NMPC all of it
   (98.5 s lap against 72.2 s), so a takeover starts with a large speed error and
   `k_speed` turns it into full braking.
4. **Yaw-rate feedback in the steering**, standing in for the NMPC's sideslip limit.
5. **Smoother steering**: it runs at the ±34 deg/s rate limit most of the time
   (`tau_steer` 0.05 s); the NMPC's rate cost keeps it smoother.
