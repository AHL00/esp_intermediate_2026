#!/usr/bin/env python3
"""Plot the output of each step of joystick_map() (controller/src/control.c) against its input.

The tuning constants are read from control.c, so the plots follow the firmware.
The steps below mirror joystick_map() one-to-one; keep them in sync if it changes.

Usage:
    python3 joystick_plot.py              # open an interactive window
    python3 joystick_plot.py -o out.png   # save to a file instead (works headless)
"""
import argparse
import re
from pathlib import Path

import matplotlib

import numpy as np

C_SOURCE = Path(__file__).resolve().parent.parent / "src" / "control.c"


def load_constants(path):
    """Pull the JOYSTICK_* #defines out of control.c."""
    text = path.read_text()
    consts = {}
    for name in ("MIN_MV", "MAX_MV", "DEADZONE", "CURVE_CONSTANT"):
        m = re.search(rf"#define\s+JOYSTICK_{name}\s+(-?[\d.]+)f?", text)
        if not m:
            raise SystemExit(f"JOYSTICK_{name} not found in {path}")
        consts[name] = float(m.group(1))
    return consts


def magnitude(x, y):
    return np.sqrt(x * x + y * y)


def stages(mv_x, mv_y, c):
    """Return [(title, x, y), ...]: the (x, y) output after each step of joystick_map()."""
    span = c["MAX_MV"] - c["MIN_MV"]
    dz = c["DEADZONE"]

    # 1. Rescale to [-1, 1]
    x = (mv_x - c["MIN_MV"]) / span * 2.0 - 1.0
    y = (mv_y - c["MIN_MV"]) / span * 2.0 - 1.0
    out = [("1. Rescale to [-1, 1]", x, y)]

    # 2. Circular deadzone, rescaled so the output starts from 0 at the edge of the deadzone
    mag = magnitude(x, y)
    inside = mag < dz
    k = np.where(inside, 0.0, (mag - dz) / (1.0 - dz) / np.where(mag == 0, 1.0, mag))
    x, y = x * k, y * k
    out.append((f"2. Deadzone ({dz:g})", x, y))

    # 3. Clamp to unit length
    mag = magnitude(x, y)
    k = np.where(mag > 1.0, 1.0 / np.where(mag == 0, 1.0, mag), 1.0)
    x, y = x * k, y * k
    out.append(("3. Clamp to unit length", x, y))

    # 4. Acceleration curve
    mag = magnitude(x, y)
    k = c["CURVE_CONSTANT"] * mag**3 + (1.0 - c["CURVE_CONSTANT"])
    x, y = x * k, y * k
    out.append((f"4. Curve (c = {c['CURVE_CONSTANT']:g})", x, y))

    return out


def plot(c, grid_points=41):
    import matplotlib.pyplot as plt

    # Palette: blue for the stage being shown, grey for the stage before it
    blue, grey, ink = "#2a78d6", "#a9a8a2", "#0b0b0b"

    lo, hi, mid = c["MIN_MV"], c["MAX_MV"], (c["MIN_MV"] + c["MAX_MV"]) / 2
    sweep = np.linspace(lo, hi, 801)
    centre = np.full_like(sweep, mid)

    x_sweep = stages(sweep, centre, c)  # X moves, Y held at centre
    diag_sweep = stages(sweep, sweep, c)  # X and Y move together (stick pushed into a corner)
    g = np.linspace(lo, hi, grid_points)
    gx, gy = np.meshgrid(g, g)
    grid = stages(gx.ravel(), gy.ravel(), c)  # whole input square

    n = len(x_sweep)
    fig, axes = plt.subplots(3, n, figsize=(4 * n, 11.5), constrained_layout=True)
    fig.suptitle("joystick_map(): output after each step vs. input", fontsize=15, color=ink)

    for i in range(n):
        title = x_sweep[i][0]

        # Rows 1 and 2: output X against input mV
        for row, data in ((0, x_sweep), (1, diag_sweep)):
            ax = axes[row, i]
            if i > 0:
                ax.plot(sweep, data[i - 1][1], color=grey, lw=1.5, label="previous step")
            ax.plot(sweep, data[i][1], color=blue, lw=2, label="this step")
            ax.set_xlim(lo, hi)
            ax.set_ylim(-1.1, 1.1)
            ax.set_xlabel("input (mV)" if row == 0 else "input, X and Y (mV)")
            ax.axhline(0, color=grey, lw=0.6)
            ax.axvline(mid, color=grey, lw=0.6)

        # Row 3: where the whole input square lands in the output plane
        ax = axes[2, i]
        ax.add_patch(plt.Circle((0, 0), 1.0, fill=False, ls="--", color=grey, lw=1))
        ax.scatter(grid[i][1], grid[i][2], s=6, color=blue, linewidths=0)
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect("equal")
        ax.set_xlabel("output X")
        ax.axhline(0, color=grey, lw=0.6)
        ax.axvline(0, color=grey, lw=0.6)

        axes[0, i].set_title(title, fontsize=12, color=ink, loc="left")

    axes[0, 0].set_ylabel("X sweep, Y centred\noutput X")
    axes[1, 0].set_ylabel("Diagonal sweep, X = Y\noutput X")
    axes[2, 0].set_ylabel("Whole input square\noutput Y")
    axes[0, 1].legend(loc="upper left", frameon=False, fontsize=9)
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", help="save the figure to this file instead of showing it")
    ap.add_argument("--source", type=Path, default=C_SOURCE, help="path to control.c")
    args = ap.parse_args()

    if args.output:
        matplotlib.use("Agg")
    consts = load_constants(args.source)
    fig = plot(consts)
    if args.output:
        fig.savefig(args.output, dpi=130)
        print(f"saved {args.output}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
