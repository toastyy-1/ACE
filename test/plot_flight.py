#!/usr/bin/env python3
# plots a rocket's exported flight data (data/<name>.csv) over time
# usage: python3 test/plot_flight.py (prompts for which CSV to plot)

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

EARTH_RADIUS = 6378137.0  # matches src/constants.hpp

REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = REPO_ROOT / "data"

AXIS_COLORS = {"x": "tab:red", "y": "tab:green", "z": "tab:blue", "w": "tab:gray"}


def plot_components(ax, d, prefix, comps, title, ylabel, scale=1.0):
    # one line per component (x/y/z, plus w for the quaternion) on the same axes
    for c in comps:
        ax.plot(d["t"], d[prefix + c] * scale, label=c, color=AXIS_COLORS[c])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.legend(loc="best")


def mark_staging(axes, d):
    # dashed vertical line wherever the active stage index changes
    if "stage" not in d.dtype.names:
        return
    changes = np.nonzero(np.diff(d["stage"]))[0] + 1
    for ax in axes.flat:
        for i in changes:
            ax.axvline(d["t"][i], color="k", linestyle="--", linewidth=0.8, alpha=0.5)


def plot_forces_and_aero(d, csv_name):
    # second figure: acceleration breakdown, aerodynamics, and stability
    t = d["t"]

    fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
    fig.suptitle(f"Forces and aerodynamics: {csv_name}")

    plot_components(axes[0, 0], d, "g", "xyz", "Gravitational acceleration (ECI)", "m/s²")
    plot_components(axes[0, 1], d, "drag", "xyz", "Drag acceleration (ECI)", "m/s²")
    plot_components(axes[1, 0], d, "thrust_a", "xyz", "Thrust acceleration (ECI)", "m/s²")
    plot_components(axes[1, 1], d, "a_spec_", "xyz", "Specific force (body, accelerometer)", "m/s²")

    ax = axes[2, 0]
    ax.plot(t, d["mach"], color="tab:blue", label="Mach")
    ax.set_title("Mach and dynamic pressure")
    ax.set_ylabel("Mach")
    ax_q = ax.twinx()
    ax_q.plot(t, d["dyn_pressure"] / 1000.0, color="tab:orange", label="q")
    ax_q.set_ylabel("q (kPa)")
    i_max_q = np.argmax(d["dyn_pressure"])
    ax_q.plot(t[i_max_q], d["dyn_pressure"][i_max_q] / 1000.0, "o", color="tab:orange")
    ax_q.annotate(f"max q {d['dyn_pressure'][i_max_q] / 1000.0:.1f} kPa @ {t[i_max_q]:.1f} s",
                  (t[i_max_q], d["dyn_pressure"][i_max_q] / 1000.0),
                  textcoords="offset points", xytext=(5, 5), fontsize=8)
    lines = ax.get_lines() + ax_q.get_lines()[:1]
    ax.legend(lines, [l.get_label() for l in lines], loc="best")

    ax = axes[2, 1]
    ax.plot(t, np.degrees(d["aoa"]), color="tab:purple")
    ax.set_title("Angle of attack")
    ax.set_ylabel("deg")

    ax = axes[3, 0]
    ax.plot(t, d["z_cm"], label="CoM", color="tab:brown")
    ax.plot(t, d["z_cp"], label="CoP", color="tab:cyan")
    ax.plot(t, d["z_cm"] - d["z_cp"], label="margin (CoM - CoP)", color="tab:gray")
    ax.axhline(0, color="k", linewidth=0.8)
    ax.set_title("Stability (from active stage aft edge)")
    ax.set_ylabel("m")
    ax.legend(loc="best")

    # fuel left in whichever stage is currently active (jumps to the next stage's load at staging)
    ax = axes[3, 1]
    if "m_fuel_stage" in d.dtype.names:
        ax.plot(t, d["m_fuel_stage"], color="tab:orange")
    else:
        ax.text(0.5, 0.5, "re-export CSV for m_fuel_stage", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("Active stage fuel")
    ax.set_ylabel("kg")

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("time (s)")
    mark_staging(axes, d)

    fig.tight_layout()


def prompt_for_csv():
    # lists the exported CSVs, then asks until it gets a file that exists
    # accepts a full path, or just a name from data/ (with or without .csv)
    available = sorted(DATA_DIR.glob("*.csv"))
    if available:
        print("available in data/:")
        for f in available:
            print(f"  {f.name}")

    while True:
        name = input("file to plot: ").strip()
        if not name:
            continue
        for candidate in (Path(name), DATA_DIR / name, DATA_DIR / f"{name}.csv"):
            if candidate.is_file():
                return candidate
        print(f"couldn't find '{name}', try again")


def main():
    try:
        csv_path = prompt_for_csv()
    except (KeyboardInterrupt, EOFError):
        print()
        sys.exit(1)
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = d["t"]

    # older CSVs don't have the exported altitude or the force/aero columns
    has_extended = "altitude" in d.dtype.names
    if has_extended:
        altitude = d["altitude"]
        altitude_title = "Altitude (above terrain)"
    else:
        r = np.column_stack((d["rx"], d["ry"], d["rz"]))
        altitude = np.linalg.norm(r, axis=1) - EARTH_RADIUS
        altitude_title = "Altitude (above spherical Earth)"

    fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
    fig.suptitle(f"Flight data: {csv_path.name}")

    plot_components(axes[0, 0], d, "r", "xyz", "Position (ECI)", "km", scale=1e-3)
    plot_components(axes[0, 1], d, "v", "xyz", "Velocity (ECI)", "m/s")
    plot_components(axes[1, 0], d, "a", "xyz", "Acceleration", "m/s²")
    plot_components(axes[1, 1], d, "w", "xyz", "Angular velocity", "rad/s")
    plot_components(axes[2, 0], d, "q", "wxyz", "Attitude quaternion", "")

    ax = axes[2, 1]
    ax.plot(t, altitude / 1000.0, color="tab:purple")
    ax.set_title(altitude_title)
    ax.set_ylabel("km")

    ax = axes[3, 0]
    ax.plot(t, d["m"], label="total mass", color="tab:brown")
    ax.plot(t, d["m_fuel"], label="fuel mass", color="tab:orange")
    ax.set_title("Mass")
    ax.set_ylabel("kg")
    ax.legend(loc="best")

    ax = axes[3, 1]
    ax.plot(t, d["thrust"] / 1000.0, color="tab:red")
    ax.set_title("Thrust")
    ax.set_ylabel("kN")

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("time (s)")
    mark_staging(axes, d)

    fig.tight_layout()

    if has_extended:
        plot_forces_and_aero(d, csv_path.name)

    plt.show()


if __name__ == "__main__":
    main()
