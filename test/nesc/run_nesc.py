#!/usr/bin/env python3
# runs every NESC check-case that has a config/ folder and compares it against the NESC simulations
# usage: python3 test/nesc/run_nesc.py [case folder name ...] [--plot]
#
# each case runs as its own headless sim instance from its folder, built with a flight controller
# that never commands anything, or with schedule_fc if the case has a config/fc_schedule.txt.
# results land in <case>/data/
#
# --plot shows a figure per case and saves it to <case>/compare.png. each quantity gets the NESC
# runs with ACE overlaid, and next to it ACE minus the NESC mean inside the band the NESC runs span

import csv
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
NESC_DIR = Path(__file__).resolve().parent
BINARIES = {"dumb_fc.cpp": REPO_ROOT / "build" / "nesc-headless",
            "schedule_fc.cpp": REPO_ROOT / "build" / "nesc-headless-schedule"}

# must match src/planetary_constants.hpp
EARTH_RATE = 7.292115e-5
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
FT = 0.3048

# compared quantities: NESC column, label
ATMOS_QUANTITIES = [
    ("altitudeMsl_ft", "altitude (ft)"),
    ("latitude_deg", "latitude (deg)"),
    ("longitude_deg", "longitude (deg)"),
    ("feVelocity_ft_s_X", "V north (ft/s)"),
    ("feVelocity_ft_s_Y", "V east (ft/s)"),
    ("feVelocity_ft_s_Z", "V down (ft/s)"),
]
ORBIT_QUANTITIES = [
    ("eiPosition_m_X", "J2000 x (m)"),
    ("eiPosition_m_Y", "J2000 y (m)"),
    ("eiPosition_m_Z", "J2000 z (m)"),
    ("eiVelocity_m_s_X", "J2000 vx (m/s)"),
    ("eiVelocity_m_s_Y", "J2000 vy (m/s)"),
    ("eiVelocity_m_s_Z", "J2000 vz (m/s)"),
]


def build(fc):
    binary = BINARIES[fc]
    subprocess.run(["make", "headless", f"FC_SRC=test/nesc/{fc}", f"HEADLESS_TARGET={binary.relative_to(REPO_ROOT)}"],
                   cwd=REPO_ROOT, check=True)
    return binary


def case_fc(case_dir):
    return "schedule_fc.cpp" if (case_dir / "config" / "fc_schedule.txt").exists() else "dumb_fc.cpp"


def epoch_era(case_dir):
    # earth rotation angle at the case's epoch, same formula as src/sim/src/ephemeris.cpp
    m = re.search(r'epoch:\s*"(\d+)-(\d+)-(\d+)T(\d+):(\d+):([\d.]+)"', (case_dir / "config" / "sim.yaml").read_text())
    y, mo, d, h, mi, sec = (float(g) for g in m.groups()) if m else (2007, 11, 20, 0, 0, 0)
    if mo <= 2:
        y, mo = y - 1, mo + 12
    a = int(y // 100)
    days = (math.floor(365.25 * (y + 4716)) + math.floor(30.6001 * (mo + 1)) + d + 2 - a + a // 4 - 1524.5
            + (h + mi / 60.0 + sec / 3600.0) / 24.0 - 2451545.0)
    return 2.0 * math.pi * (0.7790572732640 + 1.00273781191135448 * days)


def read_csv(path):
    # columns as float arrays, anything that isn't a number becomes nan
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for key in rows[0]:
        if not key:
            continue
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        cols[key.strip()] = np.array(vals)
    return cols


def geodetic(x, y, z):
    # ECEF to WGS-84 geodetic latitude (rad), longitude (rad), and height (m)
    e2 = WGS84_F * (2.0 - WGS84_F)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - e2))
    for _ in range(5):
        N = WGS84_A / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1.0 - e2 * N / (N + h)))
    N = WGS84_A / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
    return lat, np.arctan2(y, x), p / np.cos(lat) - N


def orbit_quantities(d, era):
    # the sim's ECI rotated back into J2000
    c, s = math.cos(era), math.sin(era)
    q = {}
    for col, key in (("r", "eiPosition_m_"), ("v", "eiVelocity_m_s_")):
        x, y, z = d[col + "x"], d[col + "y"], d[col + "z"]
        q[key + "X"], q[key + "Y"], q[key + "Z"] = c * x - s * y, s * x + c * y, z
    return d["t"], q


def sim_quantities(d):
    # the sim's ECI state turned into the NESC quantities
    t = d["t"]
    c, s = np.cos(EARTH_RATE * t), np.sin(EARTH_RATE * t)
    rx, ry, rz = d["rx"], d["ry"], d["rz"]

    # ECI is ECEF at t = 0, rotate back by the earth's spin
    x, y, z = c * rx + s * ry, -s * rx + c * ry, rz
    vx_rel = d["vx"] + EARTH_RATE * ry
    vy_rel = d["vy"] - EARTH_RATE * rx
    vx, vy, vz = c * vx_rel + s * vy_rel, -s * vx_rel + c * vy_rel, d["vz"]

    lat, lon, h = geodetic(x, y, z)
    sl, cl, so, co = np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon)
    return t, {
        "altitudeMsl_ft": h / FT,
        "latitude_deg": np.degrees(lat),
        "longitude_deg": np.degrees(lon),
        "feVelocity_ft_s_X": (-sl * co * vx - sl * so * vy + cl * vz) / FT,
        "feVelocity_ft_s_Y": (-so * vx + co * vy) / FT,
        "feVelocity_ft_s_Z": (-cl * co * vx - cl * so * vy - sl * vz) / FT,
    }


def plot_quantity(ax_val, ax_diff, label, t, ours, nesc, key, stack):
    # left: every NESC run with ACE dashed on top. right: offsets from the NESC mean
    for n, c in nesc.items():
        if key in c:
            ax_val.plot(c["time"], c[key], linewidth=0.8, label=n)
    ax_val.plot(t, ours, "k--", linewidth=1.2, label="ACE")
    ax_val.set_title(label)
    ax_val.legend(fontsize=6)

    mean = stack.mean(axis=0)
    ax_diff.fill_between(t, stack.min(axis=0) - mean, stack.max(axis=0) - mean, color="tab:blue", alpha=0.25, label="NESC spread")
    ax_diff.plot(t, ours - mean, "k", linewidth=1.0, label="ACE")
    ax_diff.axhline(0.0, color="gray", linewidth=0.6)
    ax_diff.set_title(f"{label}, minus NESC mean")
    ax_diff.legend(fontsize=6)


def compare(case_dir, binary, plot):
    print(f"\n== {case_dir.name}", flush=True)
    subprocess.run([str(binary)], cwd=case_dir, check=True, stdout=subprocess.DEVNULL)

    ours = read_csv(sorted((case_dir / "data").glob("*.csv"))[0])
    if case_dir.parent.name == "orbit":
        quantities = ORBIT_QUANTITIES
        t, q = orbit_quantities(ours, epoch_era(case_dir))
    else:
        quantities = ATMOS_QUANTITIES
        t, q = sim_quantities(ours)
    nesc = {p.stem: read_csv(p) for p in sorted(case_dir.glob("*_sim_*.csv"))}

    bad = np.isnan(q[quantities[0][0]])
    if bad.any():
        print(f"   ACE output turned to nan at t = {t[bad][0]:g} s, only the rows before are compared")
    print(f"   {'quantity':<16}{'max |sim - NESC mean|':>24}{'NESC spread':>14}")
    if plot:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 4, figsize=(20, 10), sharex=True)
        fig.suptitle(case_dir.name)

    for i, (key, label) in enumerate(quantities):
        # every NESC simulation that logged this quantity, sampled at the sim's times
        runs = {n: np.interp(t, c["time"], c[key]) for n, c in nesc.items() if key in c and not np.isnan(c[key]).all()}
        if not runs:
            continue
        stack = np.array(list(runs.values()))
        diff = np.nanmax(np.abs(q[key] - stack.mean(axis=0)))
        spread = np.max(stack.max(axis=0) - stack.min(axis=0))
        print(f"   {label:<16}{diff:>24.6g}{spread:>14.6g}")

        if plot:
            row, col = divmod(i, 2)
            plot_quantity(axes[row, 2 * col], axes[row, 2 * col + 1], label, t, q[key], nesc, key, stack)

    if plot:
        for ax in axes[-1]:
            ax.set_xlabel("time (s)")
        fig.tight_layout()
        fig.savefig(case_dir / "compare.png", dpi=120)


def main():
    args = [a for a in sys.argv[1:] if a != "--plot"]
    cases = sorted(p.parent.parent for p in NESC_DIR.glob("*/*/config/rocket.yaml"))
    if args:
        cases = [c for c in cases if c.name in args]

    plot = "--plot" in sys.argv
    binaries = {fc: build(fc) for fc in sorted({case_fc(c) for c in cases})}
    for case_dir in cases:
        compare(case_dir, binaries[case_fc(case_dir)], plot)

    if plot:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
