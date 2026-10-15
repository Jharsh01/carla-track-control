"""Fit the bicycle model's tires to the Project Chrono sedan, one axle at a time.

The car is driven in Chrono at constant speed while the steering angle ramps
slowly up (and, separately, down), on ground of friction 0.9 (the race track's
asphalt). Each step gives the two axle lateral forces from the quasi-steady
force and moment balance

    m (dUy/dt + r Ux) = Fyf cos(delta) + Fyr,     Izz dr/dt = a Fyf cos(delta) - b Fyr

and the two slip angles from the kinematics. A Fiala curve (the model's tire,
track_control/model/tire.hpp) is then fitted to each axle's points by least
squares: its cornering stiffness and its peak friction. The front peak is
written as grip_f relative to the rear's, which becomes mu.

Why: one friction coefficient for both axles, identified from where the car
stops gaining lateral acceleration, makes the model understeer at the limit
while Chrono's sedan is close to neutral and its rear lets go first. An NMPC
planning with that model drives the Chrono car into a spin.

    python scripts/fit_chrono_tires.py              fit and write config/vehicles/chrono_sedan.json
    python scripts/fit_chrono_tires.py --dry-run    fit and print only

Needs the trackctl module built with Chrono (setup.sh, or cmake with Chrono found).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "build", "python"))
import trackctl  # noqa: E402

PARAMS = os.path.join(REPO, "config", "vehicles", "chrono_sedan.json")
GROUND_MU = 0.9
SPEEDS = (10.0, 14.0, 18.0, 22.0)  # [m/s]
RAMP = 0.01  # steering rate [rad/s]
DT = 0.02  # [s]
SIDESLIP_STOP = math.radians(8.0)  # end a ramp once the car slides this much


def collect(p):
    """Slip angles and axle forces (alpha_f, Fyf, alpha_r, Fyr), signs as the model's."""
    plant = trackctl.ChronoPlant(p)
    L = p.a + p.b
    rows = []
    for speed in SPEEDS:
        for sign in (1.0, -1.0):
            x0 = np.zeros(7)
            x0[3] = speed
            plant.reset(x0)
            plant.set_friction(GROUND_MU)
            x = plant.observe()
            prev = x.copy()
            for _ in range(int(30.0 / DT)):
                drag = 0.5 * p.rho * p.CdA * x[3] ** 2 + p.Crr * p.m * p.g
                x = plant.step(np.array([sign * RAMP, p.m * 2.0 * (speed - x[3]) + drag]), DT)
                dUy, dr = (x[4] - prev[4]) / DT, (x[5] - prev[5]) / DT
                prev = x.copy()
                Ux, Uy, r, delta = x[3], x[4], x[5], x[6]
                if abs(math.atan2(Uy, Ux)) > SIDESLIP_STOP or Ux < 0.8 * speed:
                    break
                ay = dUy + r * Ux
                Fyr = (p.m * ay * p.a - p.Izz * dr) / L
                Fyf = (p.m * ay - Fyr) / math.cos(delta)
                alpha_f = math.atan((Uy + p.a * r) / Ux) - delta
                alpha_r = math.atan((Uy - p.b * r) / Ux)
                rows.append((alpha_f, Fyf, alpha_r, Fyr))
    return np.array(rows)


def fiala(alpha, C, F_max):
    z = np.tan(alpha)
    grip = -C * z + C ** 2 / (3 * F_max) * np.abs(z) * z - C ** 3 / (27 * F_max ** 2) * z ** 3
    return np.where(np.abs(z) < 3 * F_max / C, grip, -F_max * np.sign(z))


def fit(alpha, F, Fz, C0):
    """Least-squares (C, mu) on a coarse grid, then a finer one around the best."""
    C_range, mu_range = (0.5 * C0, 1.5 * C0), (0.6, 1.5)
    best = None
    for _ in range(3):
        for C in np.linspace(*C_range, 41):
            for mu in np.linspace(*mu_range, 41):
                err = float(np.mean((fiala(alpha, C, mu * Fz) - F) ** 2))
                if best is None or err < best[0]:
                    best = (err, C, mu)
        dC, dmu = (C_range[1] - C_range[0]) / 20, (mu_range[1] - mu_range[0]) / 20
        C_range, mu_range = (best[1] - dC, best[1] + dC), (best[2] - dmu, best[2] + dmu)
    return best[1], best[2], math.sqrt(best[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dry-run", action="store_true", help="print the fit, do not write the file")
    args = parser.parse_args()

    if not trackctl.have_chrono:
        sys.exit("trackctl was built without Project Chrono")
    p = trackctl.VehicleParams.from_file(PARAMS)
    rows = collect(p)
    L = p.a + p.b
    Fzf, Fzr = p.m * p.g * p.b / L, p.m * p.g * p.a / L
    Cf, mu_f, rms_f = fit(rows[:, 0], rows[:, 1], Fzf, p.C_alpha_f)
    Cr, mu_r, rms_r = fit(rows[:, 2], rows[:, 3], Fzr, p.C_alpha_r)
    print(f"{len(rows)} points, ground friction {GROUND_MU}")
    print(f"front: C_alpha {Cf:8.0f} N/rad, peak friction {mu_f:.3f}, rms error {rms_f:4.0f} N"
          f"   (was {p.C_alpha_f:.0f}, {p.mu * p.grip_f:.3f})")
    print(f"rear:  C_alpha {Cr:8.0f} N/rad, peak friction {mu_r:.3f}, rms error {rms_r:4.0f} N"
          f"   (was {p.C_alpha_r:.0f}, {p.mu * p.grip_r:.3f})")
    if args.dry_run:
        return

    with open(PARAMS) as f:
        data = json.load(f)
    q = data["vehicle"]["parameters"]
    source = "identified"
    how = f"Fiala fit to steering ramps at {', '.join(f'{v:.0f}' for v in SPEEDS)} m/s on ground {GROUND_MU} (scripts/fit_chrono_tires.py)"
    q["C_alpha_f"] = {"value": Cf, "unit": "N/rad", "source": source, "description": "front axle cornering stiffness; " + how}
    q["C_alpha_r"] = {"value": Cr, "unit": "N/rad", "source": source, "description": "rear axle cornering stiffness; " + how}
    q["mu"] = {"value": mu_r, "unit": "-", "source": source,
               "description": "peak friction of the rear tires on ground of coefficient 0.9 (the dry road); " + how}
    # keep the key order: grip_f and grip_r right after mu
    rebuilt = {}
    for key, value in q.items():
        if key in ("grip_f", "grip_r"):
            continue
        rebuilt[key] = value
        if key == "mu":
            rebuilt["grip_f"] = {"value": mu_f / mu_r, "unit": "-", "source": source,
                                 "description": "peak friction of the front tires relative to mu; " + how}
            rebuilt["grip_r"] = {"value": 1.0, "unit": "-", "source": "design", "description": "the rear tires define mu"}
    data["vehicle"]["parameters"] = rebuilt
    with open(PARAMS, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    print(f"wrote {os.path.relpath(PARAMS, REPO)}")


if __name__ == "__main__":
    main()
