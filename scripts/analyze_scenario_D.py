#!/usr/bin/env python3
"""ARGUS analyze_scenario_D.py

Analyze the Scenario D ("lights-off") ablation: C1 (recovery OFF, flies blind) vs
C3 (recovery ON, holds during the blackout). Reads each run's recorded bag
(/argus/vio/health, /argus/health/recovery_active, /argus/ground_truth/pose,
/argus/vio/odom), and produces:

  * per-run health metrics (status %, time-in-LOST, recovery activations);
  * a VIO-vs-GT drift estimate through the run (translation-aligned at the first
    VIO sample -- valid for this straight +x flight where the VINS world frame is
    aligned with world ENU);
  * a 2-row comparison figure (status timeline + GT-x + blackout/recovery spans);
  * scenario_D_metrics.json for the ablation table / dashboard.

Run with the eval interpreter:
  ~/.venvs/argus-eval/bin/python scripts/analyze_scenario_D.py
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path.home() / "argus/data/eval/scenario_D"
STATUS_NAME = {0: "INITIALIZING", 1: "NOMINAL", 2: "DEGRADED", 3: "LOST"}
STATUS_COLOR = {0: "#9e9e9e", 1: "#2e7d32", 2: "#f9a825", 3: "#c62828"}


def _typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    msg = (Path.home() / "argus/src/argus_msgs/msg/VIOHealth.msg").read_text()
    ts.register(get_types_from_msg(msg, "argus_msgs/msg/VIOHealth"))
    return ts


def read_run(bag_dir: Path, ts) -> dict:
    health, recov, gt, vio = [], [], [], []
    with Reader(bag_dir) as r:
        for conn, t_ns, raw in r.messages():
            t = t_ns * 1e-9
            topic = conn.topic
            if topic == "/argus/vio/health":
                m = ts.deserialize_cdr(raw, conn.msgtype)
                health.append((t, m.status, m.confidence, m.num_inlier_features,
                               m.estimated_drift_rate))
            elif topic == "/argus/health/recovery_active":
                m = ts.deserialize_cdr(raw, conn.msgtype)
                recov.append((t, bool(m.data)))
            elif topic == "/argus/ground_truth/pose":
                m = ts.deserialize_cdr(raw, conn.msgtype)
                p = m.pose.position
                gt.append((t, p.x, p.y, p.z))
            elif topic == "/argus/vio/odom":
                m = ts.deserialize_cdr(raw, conn.msgtype)
                p = m.pose.pose.position
                vio.append((t, p.x, p.y, p.z))
    for arr in (health, recov, gt, vio):
        arr.sort(key=lambda r: r[0])
    return {"health": health, "recov": recov, "gt": gt, "vio": vio}


def _interp(series, t, idx):
    """Nearest-sample lookup of column idx in a time-sorted series at time t."""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    if t <= series[0][0]:
        return series[0][idx]
    if t >= series[-1][0]:
        return series[-1][idx]
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo][idx]


def rising_edges(recov) -> int:
    n, prev = 0, False
    for _t, v in recov:
        if v and not prev:
            n += 1
        prev = v
    return n


def time_in(recov_or_health, pred) -> float:
    """Integrate dwell time where pred(sample) is true (trapezoid on timestamps)."""
    total = 0.0
    for i in range(1, len(recov_or_health)):
        dt = recov_or_health[i][0] - recov_or_health[i - 1][0]
        if 0 < dt < 2.0 and pred(recov_or_health[i - 1]):
            total += dt
    return total


def vio_drift(gt, vio) -> dict:
    """Translation-align VIO to GT at the first VIO sample, then report error."""
    if len(vio) < 5 or len(gt) < 5:
        return {"max_m": None, "final_m": None, "mean_m": None}
    t0 = vio[0][0]
    gx, gy, gz = (_interp(gt, t0, 1), _interp(gt, t0, 2), _interp(gt, t0, 3))
    ox, oy, oz = gx - vio[0][1], gy - vio[0][2], gz - vio[0][3]
    errs = []
    for t, x, y, z in vio:
        gxi, gyi, gzi = _interp(gt, t, 1), _interp(gt, t, 2), _interp(gt, t, 3)
        ex, ey, ez = (x + ox) - gxi, (y + oy) - gyi, (z + oz) - gzi
        errs.append((ex * ex + ey * ey + ez * ez) ** 0.5)
    return {"max_m": max(errs), "final_m": errs[-1],
            "mean_m": sum(errs) / len(errs)}


def metrics(run: dict) -> dict:
    health = run["health"]
    if not health:
        return {}
    n = len(health)
    dist = {STATUS_NAME[s]: sum(1 for r in health if r[1] == s) / n
            for s in (0, 1, 2, 3)}
    return {
        "health_msgs": n,
        "pct_nominal": round(100 * dist["NOMINAL"], 1),
        "pct_degraded": round(100 * dist["DEGRADED"], 1),
        "pct_lost": round(100 * dist["LOST"], 1),
        "time_in_lost_s": round(time_in(health, lambda r: r[1] == 3), 2),
        "recovery_activations": rising_edges(run["recov"]),
        "time_recovery_active_s": round(time_in(run["recov"], lambda r: r[1]), 2),
        "min_confidence": round(min(r[2] for r in health), 3),
        "vio_drift": {k: (round(v, 3) if v is not None else None)
                      for k, v in vio_drift(run["gt"], run["vio"]).items()},
    }


def plot(runs: dict, out: Path) -> None:
    fig, axes = plt.subplots(len(runs), 1, figsize=(12, 3.2 * len(runs)), sharex=False)
    if len(runs) == 1:
        axes = [axes]
    for ax, (name, run) in zip(axes, runs.items()):
        health, recov, gt = run["health"], run["recov"], run["gt"]
        if not health:
            ax.set_title(f"{name}: no data"); continue
        t0 = health[0][0]
        ht = [r[0] - t0 for r in health]
        hs = [r[1] for r in health]
        # status step (left axis)
        ax.step(ht, hs, where="post", color="black", lw=1.0, zorder=3)
        ax.fill_between(ht, hs, step="post", alpha=0.10, color="black", zorder=1)
        ax.set_yticks([0, 1, 2, 3])
        ax.set_yticklabels(["INIT", "NOM", "DEG", "LOST"])
        ax.set_ylim(-0.3, 3.3)
        ax.set_ylabel("VIO health")
        # recovery-active spans (orange)
        in_span = False; start = 0.0
        for t, v in recov:
            if v and not in_span:
                in_span = True; start = t - t0
            elif not v and in_span:
                in_span = False
                ax.axvspan(start, t - t0, color="#fb8c00", alpha=0.18, zorder=0)
        if in_span:
            ax.axvspan(start, ht[-1], color="#fb8c00", alpha=0.18, zorder=0)
        # GT x position (right axis)
        if gt:
            axr = ax.twinx()
            axr.plot([g[0] - t0 for g in gt], [g[1] for g in gt],
                     color="#1565c0", lw=1.3, zorder=2)
            axr.set_ylabel("GT x (m)", color="#1565c0")
            axr.tick_params(axis="y", labelcolor="#1565c0")
        m = metrics(run)
        ax.set_title(
            f"{name.upper()}  —  NOMINAL {m['pct_nominal']}% / DEGRADED {m['pct_degraded']}% "
            f"/ LOST {m['pct_lost']}%   |   recoveries={m['recovery_activations']}, "
            f"time-in-LOST={m['time_in_lost_s']}s, VIO max-drift={m['vio_drift']['max_m']} m",
            fontsize=9.5)
        ax.set_xlabel("time (s)")
        ax.grid(True, axis="x", alpha=0.3)
    legend = [Patch(facecolor="#fb8c00", alpha=0.3, label="recovery active"),
              Patch(facecolor="#1565c0", label="GT x-position")]
    axes[0].legend(handles=legend, loc="upper left", fontsize=8)
    fig.suptitle("ARGUS Scenario D (lights-off): health monitor + recovery ablation",
                 fontsize=12, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    import sys
    ts = _typestore()
    # Discover run dirs (each has a bag/ subdir). Preferred display order: the
    # clean OFFLINE replays first (full VINS budget), then the live runs.
    wanted = sys.argv[1:] if len(sys.argv) > 1 else [
        "offline_c3", "offline_c1", "c3", "c1"]
    runs: dict[str, dict] = {}
    for mode in wanted:
        bag = ROOT / mode / "bag"
        if bag.is_dir():
            runs[mode] = read_run(bag, ts)
    if not runs:
        print(f"[scenD] no runs under {ROOT} (run replay_scenario_D.sh c3|c1 first).")
        return 1

    out_metrics = {mode: metrics(run) for mode, run in runs.items()}
    (ROOT / "scenario_D_metrics.json").write_text(json.dumps(out_metrics, indent=2))
    plot(runs, ROOT / "scenario_D_health.png")

    print("\n===== Scenario D ablation =====")
    hdr = f"{'metric':24s} " + " ".join(f"{m.upper():>12s}" for m in runs)
    print(hdr); print("-" * len(hdr))
    keys = [("pct_nominal", "NOMINAL %"), ("pct_degraded", "DEGRADED %"),
            ("pct_lost", "LOST %"), ("time_in_lost_s", "time-in-LOST s"),
            ("recovery_activations", "recoveries"),
            ("time_recovery_active_s", "time-held s"), ("min_confidence", "min conf")]
    for k, label in keys:
        print(f"{label:24s} " + " ".join(f"{out_metrics[m].get(k,''):>12}" for m in runs))
    for stat in ("max_m", "final_m", "mean_m"):
        print(f"{'VIO drift ' + stat:24s} "
              + " ".join(f"{out_metrics[m]['vio_drift'][stat]:>12}" for m in runs))
    print(f"\nfigure  -> {ROOT/'scenario_D_health.png'}")
    print(f"metrics -> {ROOT/'scenario_D_metrics.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
