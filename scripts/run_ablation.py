#!/usr/bin/env python3
"""ARGUS ablation harness.

Master plan section 9 defines the ablation grid:

    configs    C1..C5  (progressively: baseline -> +SuperPoint -> +recovery
                        -> +perception-aware planner -> +chance constraint)
    scenarios  A,B,C,D  (easy / blank-wall / loop / lights-off)
    metrics    ATE, max RPE, drift%, success rate, path length, recovery count,
               loop-closure count

This file provides:
  * the CONFIG x SCENARIO matrix structure (frozen names; downstream days fill
    in the variant wiring), and
  * a working ``aggregate`` mode that scans ``data/eval/*/metrics.json`` and
    emits a side-by-side comparison table (CSV) + bar chart -- usable today.

The per-cell *runner* (launch sim with a given config on a given scenario,
record a bag, call run_eval) is stubbed until the variants exist (Days 3-9).

Run with the eval interpreter:
    ~/.venvs/argus-eval/bin/python scripts/run_ablation.py aggregate
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


# ---------------------------------------------------------------------------
# Frozen ablation grid (names are a contract; see master plan section 9).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    """One ablation configuration (a row in the grid)."""

    name: str
    frontend: str          # "klt" | "superpoint"
    recovery: bool         # health-monitor recovery enabled
    planner: str           # "none" | "shortest" | "perception" | "cvar"
    description: str


CONFIGS: tuple[Config, ...] = (
    Config("C1", "klt", False, "none", "baseline VINS-Fusion + KLT"),
    Config("C2", "superpoint", False, "none", "+ SuperPoint front-end"),
    Config("C3", "superpoint", True, "none", "+ recovery"),
    Config("C4", "superpoint", True, "perception", "+ perception-aware planner"),
    Config("C5", "superpoint", True, "cvar", "+ chance-constrained planner (full)"),
)

# Scenarios are codified as data/scenarios/*.yaml across Days 3/5/9.
SCENARIOS: tuple[str, ...] = ("A_easy", "B_hard", "C_loop", "D_lights_off")

# Metrics surfaced in the comparison table (keys must match run_eval output).
TABLE_METRICS: tuple[str, ...] = (
    "ate_rmse_m",
    "rpe_max_m_per_m",
    "drift_pct_ate",
    "drift_pct_final",
    "path_length_m",
)


@dataclass
class Cell:
    """Result of one (config, scenario) run."""

    config: str
    scenario: str
    metrics: dict = field(default_factory=dict)
    eval_dir: Optional[Path] = None


def _run_cell(config: Config, scenario: str, out_root: Path) -> Cell:
    """Run one matrix cell end-to-end. STUB until variants/scenarios exist.

    Launch the sim on ``scenario`` with the front-end / recovery / planner
    selected by ``config``, record a bag, then call ``run_eval.py`` and load
    the resulting metrics.json.
    """
    raise NotImplementedError(
        f"matrix runner not wired yet for {config.name} x {scenario} "
        "(front-end / recovery / planner variants land Days 3-9)."
    )


def aggregate(eval_root: Path, out_dir: Path) -> int:
    """Build a comparison table + bar chart from existing eval runs."""
    runs = sorted(p.parent for p in eval_root.glob("*/metrics.json"))
    if not runs:
        print(f"[ablation] no eval runs under {eval_root} (run run_eval.py first).")
        return 1

    rows: list[Cell] = []
    for run in runs:
        metrics = json.loads((run / "metrics.json").read_text())
        rows.append(Cell(config=run.name, scenario="-", metrics=metrics, eval_dir=run))

    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "ablation_table.csv"
    with csv_path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["run", *TABLE_METRICS])
        for cell in rows:
            writer.writerow([cell.config, *(cell.metrics.get(m, "") for m in TABLE_METRICS)])

    # Side-by-side bar chart of ATE and drift% across runs.
    labels = [c.config for c in rows]
    ate = [c.metrics.get("ate_rmse_m", 0.0) or 0.0 for c in rows]
    drift = [c.metrics.get("drift_pct_ate", 0.0) or 0.0 for c in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    ax1.bar(labels, ate, color="C0")
    ax1.set_ylabel("ATE RMSE (m)")
    ax1.set_title("Absolute trajectory error")
    ax2.bar(labels, drift, color="C3")
    ax2.axhline(1.5, color="k", ls="--", label="1.5% target")
    ax2.set_ylabel("drift % (ATE / path length)")
    ax2.set_title("Drift percentage")
    ax2.legend()
    for ax in (ax1, ax2):
        ax.tick_params(axis="x", rotation=30)
        ax.grid(True, axis="y", alpha=0.3)
    fig.suptitle("ARGUS ablation comparison")
    fig.savefig(out_dir / "ablation_comparison.png", bbox_inches="tight")
    plt.close(fig)

    print(f"[ablation] {len(rows)} run(s) aggregated.")
    print(f"[ablation] table -> {csv_path}")
    print(f"[ablation] chart -> {out_dir / 'ablation_comparison.png'}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="ARGUS ablation harness (skeleton).")
    sub = parser.add_subparsers(dest="mode", required=True)

    agg = sub.add_parser("aggregate", help="aggregate existing data/eval/* runs")
    agg.add_argument("--eval-root", default=Path("data/eval"), type=Path)
    agg.add_argument("--out-dir", default=Path("data/eval/_ablation"), type=Path)

    mat = sub.add_parser("matrix", help="run the full CONFIG x SCENARIO grid (stub)")
    mat.add_argument("--out-root", default=Path("data/eval"), type=Path)

    args = parser.parse_args()

    if args.mode == "aggregate":
        return aggregate(args.eval_root, args.out_dir)

    # mode == "matrix": print the planned grid, then attempt (stubbed) runs.
    print("[ablation] planned grid (configs x scenarios):")
    for cfg in CONFIGS:
        print(f"  {cfg.name}: {cfg.description}")
    print(f"  scenarios: {', '.join(SCENARIOS)}")
    for cfg in CONFIGS:
        for scenario in SCENARIOS:
            try:
                _run_cell(cfg, scenario, args.out_root)
            except NotImplementedError as exc:
                print(f"  [skip] {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
