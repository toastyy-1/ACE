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

    r = np.column_stack((d["rx"], d["ry"], d["rz"]))
    altitude = np.linalg.norm(r, axis=1) - EARTH_RADIUS

    fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
    fig.suptitle(f"Flight data: {csv_path.name}")

    plot_components(axes[0, 0], d, "r", "xyz", "Position (ECI)", "km", scale=1e-3)
    plot_components(axes[0, 1], d, "v", "xyz", "Velocity (ECI)", "m/s")
    plot_components(axes[1, 0], d, "a", "xyz", "Acceleration", "m/s²")
    plot_components(axes[1, 1], d, "w", "xyz", "Angular velocity", "rad/s")
    plot_components(axes[2, 0], d, "q", "wxyz", "Attitude quaternion", "")

    ax = axes[2, 1]
    ax.plot(t, altitude / 1000.0, color="tab:purple")
    ax.set_title("Altitude (above spherical Earth)")
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

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
