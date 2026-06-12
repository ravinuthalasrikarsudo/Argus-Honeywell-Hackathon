#!/usr/bin/env python3
"""ARGUS :: make_scenarioE_figures.py — publication figures for the 200 m gate.

Reads the aligned gt.tum / vio.tum that run_eval.py leaves in an eval dir and
renders the two README figures for Scenario E:

  fig_scenarioE_trajectory.png  top-down (XY) circuit: GT vs VIO, coloured by
                                APE, with the tunnel stadium outline underlay
  fig_scenarioE_drift.png       absolute position error vs distance travelled
                                against the 1.5 % drift budget envelope

Optionally overlays a second eval dir (e.g. raw odom_optimized vs loop-
corrected odom_loop) on the drift figure for the loop-closure story.

Run with the eval interpreter:
  ~/.venvs/argus-eval/bin/python scripts/make_scenarioE_figures.py \
      --eval-dir data/eval/E_tunnel_loop --raw-dir data/eval/E_tunnel_raw \
      --out-dir docs/figures
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

plt.rcParams.update({
    "font.size": 13, "axes.titlesize": 15, "axes.labelsize": 14,
    "legend.fontsize": 12, "figure.dpi": 120, "savefig.bbox": "tight",
})

L, R, W = 70.0, 10.0, 3.0     # must match generate_tunnel_circuit.py


def load_tum(path: Path) -> np.ndarray:
    """t x y z qx qy qz qw rows -> (N, 4) [t, x, y, z]."""
    d = np.loadtxt(path)
    return d[:, :4]


def stadium_outline(off: float, n: int = 64):
    """Closed wall polyline: centreline offset by `off` (+W outer / -W inner)."""
    pts = []
    r = R + off
    pts += [(x, -off) for x in np.linspace(0, L, 8)]                # straight A
    ang = np.linspace(-math.pi / 2, math.pi / 2, n // 2)
    pts += [(L + r * math.cos(a), R + r * math.sin(a)) for a in ang]
    pts += [(x, 2 * R + off) for x in np.linspace(L, 0, 8)]         # straight B
    ang = np.linspace(math.pi / 2, 3 * math.pi / 2, n // 2)
    pts += [(r * math.cos(a), R + r * math.sin(a)) for a in ang]
    pts.append(pts[0])
    return np.array(pts)


def cum_dist(xyz: np.ndarray) -> np.ndarray:
    d = np.zeros(len(xyz))
    d[1:] = np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))
    return d


def ape_series(gt: np.ndarray, est: np.ndarray, max_dt: float = 0.05):
    """Associate est->gt by nearest timestamp; per-pose APE on the matches."""
    idx = np.searchsorted(gt[:, 0], est[:, 0])
    idx = np.clip(idx, 1, len(gt) - 1)
    left_closer = (est[:, 0] - gt[idx - 1, 0]) < (gt[idx, 0] - est[:, 0])
    idx[left_closer] -= 1
    ok = np.abs(gt[idx, 0] - est[:, 0]) <= max_dt
    g, e = gt[idx[ok]], est[ok]
    ape = np.linalg.norm(g[:, 1:4] - e[:, 1:4], axis=1)
    return g, e, ape


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--eval-dir", required=True,
                    help="run_eval output dir (loop-corrected headline run)")
    ap.add_argument("--raw-dir", default=None,
                    help="optional second dir (raw VIO) for the drift overlay")
    ap.add_argument("--out-dir", default="docs/figures")
    ap.add_argument("--label", default="VIO (loop-corrected)")
    ap.add_argument("--raw-label", default="VIO (raw odometry)")
    ap.add_argument("--budget-pct", type=float, default=1.5)
    a = ap.parse_args()

    ed = Path(a.eval_dir)
    gt, est, ape = ape_series(load_tum(ed / "gt.tum"), load_tum(ed / "vio.tum"))
    dist = cum_dist(gt[:, 1:4])

    # ---------------- fig 1: top-down circuit ----------------
    fig, ax = plt.subplots(figsize=(11, 5.2))
    for off, c in ((+W, "0.55"), (-W, "0.55")):
        o = stadium_outline(off)
        ax.plot(o[:, 0], o[:, 1], color=c, lw=1.2, zorder=1,
                label="tunnel walls" if off > 0 else None)
    ax.plot(gt[:, 1], gt[:, 2], "--", color="0.25", lw=1.6, zorder=2,
            label="ground truth")
    pts = est[:, 1:3].reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="turbo", zorder=3, linewidths=2.2)
    lc.set_array(ape[:-1])
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, pad=0.012)
    cb.set_label("APE (m)")
    ax.plot(gt[0, 1], gt[0, 2], "o", ms=9, mfc="lime", mec="k", zorder=4,
            label="start / loop-closure point")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.set_title(f"Scenario E — {dist[-1]:.1f} m tunnel circuit: VIO vs ground truth")
    ax.legend(loc="center")
    fig.savefig(Path(a.out_dir) / "fig_scenarioE_trajectory.png")
    plt.close(fig)

    # ---------------- fig 2: error vs distance + budget ----------------
    fig, ax = plt.subplots(figsize=(10, 4.6))
    bud = a.budget_pct / 100.0 * dist
    ax.fill_between(dist, 0, bud, color="tab:red", alpha=0.10)
    ax.plot(dist, bud, "r--", lw=1.8,
            label=f"{a.budget_pct}% drift budget")
    if a.raw_dir:
        rgt, rest, rape = ape_series(load_tum(Path(a.raw_dir) / "gt.tum"),
                                     load_tum(Path(a.raw_dir) / "vio.tum"))
        ax.plot(cum_dist(rgt[:, 1:4]), rape, color="tab:orange", lw=1.6,
                alpha=0.9, label=a.raw_label)
    ax.plot(dist, ape, color="tab:blue", lw=2.0, label=a.label)
    ax.set_xlabel("distance travelled (m)")
    ax.set_ylabel("absolute position error (m)")
    ax.set_title("Scenario E — APE vs distance against the 1.5 % budget envelope")
    ax.set_xlim(0, dist[-1] * 1.02)
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)
    fig.savefig(Path(a.out_dir) / "fig_scenarioE_drift.png")
    plt.close(fig)

    print(f"[figE] path={dist[-1]:.2f} m  ATE RMSE={np.sqrt((ape**2).mean()):.3f} m"
          f"  drift%={100*np.sqrt((ape**2).mean())/dist[-1]:.3f}"
          f"  final={ape[-1]:.3f} m -> {a.out_dir}")


if __name__ == "__main__":
    main()
