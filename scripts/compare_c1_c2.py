#!/usr/bin/env python3
"""ARGUS :: compare_c1_c2.py  -- C1 (KLT/Harris) vs C2 (SuperPoint) front-end.

Diffs the two eval matrices produced by run_day3_evals.sh (C1_klt) and
run_day5_evals.sh (C2_superpoint), 1:1 per scenario (identical slices), and emits:

  * a markdown + console table of drift (ATE%, final%, KITTI mean%) with the
    C2-vs-C1 delta and % improvement,
  * a top-down trajectory overlay per scenario (GT vs C1-VIO vs C2-VIO),
  * a grouped bar chart of ATE drift, C1 vs C2, across scenarios.

Run with the eval venv:  ~/.venvs/argus-eval/bin/python scripts/compare_c1_c2.py

NOTE: we plot manually with set_aspect('auto') + autoscale; evo's traj_colormap
forces equal aspect, which collapsed the near-straight A/B paths to an invisible
sliver.
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WS = "/home/vittal/argus"
C1 = os.path.join(WS, "data/eval/C1_klt")
C2 = os.path.join(WS, "data/eval/C2_superpoint")
OUT = os.path.join(WS, "data/eval/compare_c1_c2")
SCENARIOS = [
    ("scenario_A", "A (easy / drift gate)"),
    ("scenario_A_baselinerun", "A (baseline straight run)"),
    ("scenario_B", "B (hard / Zone-B blank walls)"),
    ("scenario_C_beforeloop", "C (loop, before)"),
    ("scenario_C_afterloop", "C (loop, after)"),
]
KEYS = ["drift_pct_ate", "drift_pct_final", "kitti_drift_pct_mean", "path_length"]


def load_metrics(root, scen):
    f = os.path.join(root, scen, "metrics.json")
    if not os.path.isfile(f):
        return None
    try:
        with open(f) as fh:
            return json.load(fh)
    except Exception as exc:  # noqa: BLE001
        print(f"  ! failed to read {f}: {exc}")
        return None


def load_tum_xy(root, scen, name):
    f = os.path.join(root, scen, name)
    if not os.path.isfile(f):
        return None
    try:
        a = np.loadtxt(f, comments="#")
        if a.ndim == 1:
            a = a[None, :]
        return a[:, 1], a[:, 2]  # x, y (columns: t x y z qx qy qz qw)
    except Exception:  # noqa: BLE001
        return None


def fmt(v):
    return "n/a" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v))


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    bar_scen, bar_c1, bar_c2 = [], [], []

    for scen, label in SCENARIOS:
        m1, m2 = load_metrics(C1, scen), load_metrics(C2, scen)
        if m1 is None and m2 is None:
            continue
        d1 = (m1 or {}).get("drift_pct_ate")
        d2 = (m2 or {}).get("drift_pct_ate")
        delta = (d2 - d1) if (d1 is not None and d2 is not None) else None
        impr = (100.0 * (d1 - d2) / d1) if (d1 and d2 is not None and d1 != 0) else None
        rows.append((label, m1, m2, delta, impr))
        if d1 is not None and d2 is not None:
            bar_scen.append(scen.replace("scenario_", "").replace("_", "\n"))
            bar_c1.append(d1)
            bar_c2.append(d2)

        # ---- trajectory overlay ----
        gt = load_tum_xy(C2, scen, "gt.tum") or load_tum_xy(C1, scen, "gt.tum")
        v1 = load_tum_xy(C1, scen, "vio.tum")
        v2 = load_tum_xy(C2, scen, "vio.tum")
        if gt or v1 or v2:
            fig, ax = plt.subplots(figsize=(7, 4))
            if gt:
                ax.plot(gt[0], gt[1], "k-", lw=2.0, label="ground truth")
            if v1:
                ax.plot(v1[0], v1[1], "b--", lw=1.3, label="C1 KLT/Harris")
            if v2:
                ax.plot(v2[0], v2[1], "-", color="tab:orange", lw=1.3, label="C2 SuperPoint")
            ax.set_title(f"ARGUS {label}: C1 vs C2 trajectory")
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_aspect("auto")
            ax.autoscale(enable=True)
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=8)
            fig.tight_layout()
            p = os.path.join(OUT, f"traj_{scen}.png")
            fig.savefig(p, dpi=130)
            plt.close(fig)

    # ---- bar chart ----
    if bar_scen:
        x = np.arange(len(bar_scen))
        w = 0.38
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.bar(x - w / 2, bar_c1, w, label="C1 KLT/Harris", color="tab:blue")
        ax.bar(x + w / 2, bar_c2, w, label="C2 SuperPoint", color="tab:orange")
        ax.set_xticks(x)
        ax.set_xticklabels(bar_scen, fontsize=8)
        ax.set_ylabel("ATE drift [%]")
        ax.set_title("ARGUS front-end ablation: ATE drift, C1 vs C2")
        ax.legend()
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "drift_bar_c1_c2.png"), dpi=130)
        plt.close(fig)

    # ---- table (console + markdown) ----
    hdr = f"{'scenario':32} | {'metric':22} | {'C1 KLT':>10} | {'C2 SP':>10} | {'delta':>9} | {'impr%':>7}"
    lines = ["# ARGUS C1 (KLT/Harris) vs C2 (SuperPoint) front-end ablation", "",
             "```", hdr, "-" * len(hdr)]
    for label, m1, m2, delta, impr in rows:
        for k in KEYS:
            v1 = (m1 or {}).get(k)
            v2 = (m2 or {}).get(k)
            dd = (v2 - v1) if (isinstance(v1, (int, float)) and isinstance(v2, (int, float))) else None
            ii = impr if k == "drift_pct_ate" else None
            lines.append(f"{label:32} | {k:22} | {fmt(v1):>10} | {fmt(v2):>10} | "
                         f"{fmt(dd):>9} | {fmt(ii):>7}")
        lines.append("-" * len(hdr))
    lines.append("```")
    lines.append("")
    lines.append("Negative delta / positive impr% on drift_pct_ate = C2 (SuperPoint) is better.")
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(OUT, "summary.md"), "w") as fh:
        fh.write(text + "\n")
    print(f"\n[compare] wrote {OUT}/summary.md + traj_*.png + drift_bar_c1_c2.png")


if __name__ == "__main__":
    main()
