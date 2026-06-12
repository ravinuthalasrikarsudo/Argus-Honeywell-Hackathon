#!/usr/bin/env python3
"""Re-render trajectory.png from saved gt.tum/vio.tum with AUTO aspect.

evo's traj plot forces set_aspect("equal"); on the near-straight
Scenario A/B runs (y/z ~= const) that squished the path to an invisible sliver.
This reads the already-aligned trajectories saved next to metrics.json and re-draws
ONLY trajectory.png (metrics.json untouched), so the documented numbers are preserved.

Usage: python _replot_trajectory.py <eval_dir> [<eval_dir> ...]
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def load_tum(path):
    a = np.loadtxt(path)
    return a[:, 0], a[:, 1:4]  # t, xyz


def ape_per_est(t_ref, xyz_ref, t_est, xyz_est):
    """Per-est-pose APE via nearest-timestamp ref match (ref is dense @250 Hz)."""
    idx = np.clip(np.searchsorted(t_ref, t_est), 0, len(t_ref) - 1)
    return np.linalg.norm(xyz_est - xyz_ref[idx], axis=1)


def colored_line(ax, x, y, c, norm):
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="jet", norm=norm)
    lc.set_array(c[:-1])
    lc.set_linewidth(2.2)
    ax.add_collection(lc)
    return lc


def replot(d):
    gt, vio = os.path.join(d, "gt.tum"), os.path.join(d, "vio.tum")
    if not (os.path.exists(gt) and os.path.exists(vio)):
        print("SKIP (no tum):", d)
        return
    t_ref, xyz_ref = load_tum(gt)
    t_est, xyz_est = load_tum(vio)
    ape = ape_per_est(t_ref, xyz_ref, t_est, xyz_est)
    norm = plt.Normalize(float(ape.min()), float(ape.max()))

    fig = plt.figure(figsize=(13, 6))
    for i, (a, b, name) in enumerate([(0, 1, "xy"), (0, 2, "xz")], start=1):
        ax = fig.add_subplot(1, 2, i)
        ax.plot(xyz_ref[:, a], xyz_ref[:, b], "--", color="0.4",
                lw=1.6, label="ground truth")
        lc = colored_line(ax, xyz_est[:, a], xyz_est[:, b], ape, norm)
        ax.set_aspect("auto")          # <-- the fix (evo forces "equal")
        ax.autoscale()
        ylo, yhi = ax.get_ylim()
        if yhi - ylo < 0.5:            # pad a degenerate (flat) axis
            mid = 0.5 * (ylo + yhi)
            ax.set_ylim(mid - 0.5, mid + 0.5)
        ax.set_title(f"trajectory ({name})")
        ax.set_xlabel(f"{name[0]} (m)")
        ax.set_ylabel(f"{name[1]} (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        fig.colorbar(lc, ax=ax, label="APE (m)")
    fig.suptitle("ARGUS VIO vs ground truth (VIO colour = APE, m)")
    fig.tight_layout()
    out = os.path.join(d, "trajectory.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"rewrote {out}  (n_est={len(t_est)} ape[min,max]="
          f"{ape.min():.3f},{ape.max():.3f})")


if __name__ == "__main__":
    for d in sys.argv[1:]:
        replot(d)
