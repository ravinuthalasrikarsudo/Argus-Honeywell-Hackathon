#!/usr/bin/env python3
"""ARGUS VIO evaluation harness (evo-based).

Reads a ROS2 rosbag containing a ground-truth pose stream and a VIO odometry
stream, aligns them (SE(3) Umeyama, *no* scale correction -- stereo-inertial VIO
is metric), and reports ATE / RPE / drift-percentage with publication-quality
plots.

IMPORTANT: run with the isolated eval interpreter, NOT system python:

    ~/.venvs/argus-eval/bin/python scripts/run_eval.py \
        --bag data/bags/<run> --run-id <name>

Dependencies (all in ~/.venvs/argus-eval): rosbags, evo, numpy, matplotlib.
The harness never imports rclpy, so it cannot disturb the ROS runtime.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Tuple

import numpy as np

import matplotlib

matplotlib.use("Agg")  # headless / WSL-safe
import matplotlib.pyplot as plt  # noqa: E402

from rosbags.highlevel import AnyReader  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

from evo.core import filters, metrics, sync  # noqa: E402
from evo.core.geometry import GeometryException  # noqa: E402
from evo.core.trajectory import PoseTrajectory3D  # noqa: E402
from evo.tools import plot as evo_plot  # noqa: E402


# Frozen ARGUS schema defaults.
DEFAULT_GT_TOPIC = "/argus/ground_truth/pose"
DEFAULT_VIO_TOPIC = "/argus/vio/odom"

# rosbag2/sqlite3 bags recorded without embedded msg definitions need an explicit
# typestore; the ARGUS schema uses only standard Humble message types.
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

# Paper-quality matplotlib defaults (consistent across all ARGUS figures).
plt.rcParams.update(
    {
        "font.size": 13,
        "axes.titlesize": 15,
        "axes.labelsize": 14,
        "legend.fontsize": 12,
        "figure.dpi": 120,
        "savefig.bbox": "tight",
    }
)


def _stamp_to_sec(msg) -> float:
    """Return the header stamp of a stamped message in float seconds."""
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def read_trajectory(bag_path: Path, topic: str) -> PoseTrajectory3D:
    """Read a stamped-pose / odometry topic from a rosbag into an evo trajectory.

    Supports geometry_msgs/PoseStamped and nav_msgs/Odometry (the two pose
    carriers in the ARGUS schema). Times come from the message *header* stamp so
    ground truth and VIO share the simulator clock.
    """
    stamps: list[float] = []
    xyz: list[list[float]] = []
    quat_wxyz: list[list[float]] = []

    with AnyReader([bag_path], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"[run_eval] topic '{topic}' not found in {bag_path}.\n"
                f"           available: {available}"
            )
        for conn, _bag_t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            # PoseStamped -> .pose.{position,orientation}; Odometry -> .pose.pose.*
            pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
            p, o = pose.position, pose.orientation
            stamps.append(_stamp_to_sec(msg))
            xyz.append([p.x, p.y, p.z])
            quat_wxyz.append([o.w, o.x, o.y, o.z])

    if len(stamps) < 2:
        raise SystemExit(
            f"[run_eval] topic '{topic}' produced {len(stamps)} poses (<2). "
            f"VIO likely did not initialize."
        )
    return PoseTrajectory3D(
        positions_xyz=np.asarray(xyz),
        orientations_quat_wxyz=np.asarray(quat_wxyz),
        timestamps=np.asarray(stamps),
    )


def _gauge_align_yaw_translation(
    ref: PoseTrajectory3D, est: PoseTrajectory3D
) -> np.ndarray:
    """4-DOF gauge alignment: global translation (x,y,z) + yaw about gravity.

    A level visual-inertial estimator with no heading reference cannot observe
    its global position or its yaw about the gravity vector -- these are gauge
    freedoms, fixed only by an arbitrary choice at initialization. Standard VIO
    benchmarks remove them with a full SE(3) Umeyama fit before computing ATE.
    On a near-straight (collinear) trajectory that Umeyama fit is rank-deficient
    and throws, so here we solve *exactly* the 4 unobservable DOF in closed form:
    align the centroids (translation) and the single yaw that best rotates `est`
    onto `ref` in the horizontal plane (2D Kabsch -- well-defined even for
    collinear points). Roll/pitch and metric scale are left untouched because
    they ARE observable (gravity from the IMU, scale from stereo). This is the
    honest gauge removal; the origin-only fallback left yaw uncorrected and
    inflated ATE roughly linearly with distance.

    Returns a 4x4 SE(3) transform mapping `est` poses into the `ref` frame.
    """
    rp = ref.positions_xyz
    ep = est.positions_xyz
    rc = rp.mean(axis=0)
    ec = ep.mean(axis=0)
    dr = rp - rc
    de = ep - ec
    # yaw minimizing || R(yaw) * de_xy - dr_xy ||^2  (closed-form 2D Kabsch)
    s = float(np.sum(de[:, 0] * dr[:, 1] - de[:, 1] * dr[:, 0]))
    c = float(np.sum(de[:, 0] * dr[:, 0] + de[:, 1] * dr[:, 1]))
    yaw = float(np.arctan2(s, c))
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    rot = np.array(
        [[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]]
    )
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = rc - rot @ ec
    return transform


def _kitti_segment_drift(ref: PoseTrajectory3D, est: PoseTrajectory3D, seg_lengths) -> dict:
    """KITTI-style translation drift: mean relative position error over fixed
    GT-distance segments, as a percentage of segment length, averaged over every
    valid start pose.

    This is the standard "drift over distance" RATE (KITTI odometry benchmark).
    It uses only RELATIVE transforms, so it is alignment-free (no Umeyama / gauge
    choice can affect it) and independent of total path length -- the faithful
    way to evaluate a "< X% over <distance>" spec on a short trajectory, since a
    fixed init/end offset averages out instead of inflating the number.
    """
    pr = ref.poses_se3
    pe = est.poses_se3
    d = np.asarray(ref.distances)  # cumulative GT path length per pose (m)
    n = len(pr)
    out = {}
    for L in seg_lengths:
        errs = []
        for i in range(n):
            j = int(np.searchsorted(d, d[i] + L))
            if j >= n:
                break
            rel_ref = np.linalg.inv(pr[i]) @ pr[j]
            rel_est = np.linalg.inv(pe[i]) @ pe[j]
            err = np.linalg.inv(rel_ref) @ rel_est
            errs.append(float(np.linalg.norm(err[:3, 3])) / L)
        if errs:
            out[int(L)] = round(100.0 * float(np.mean(errs)), 3)
    return out


def evaluate(
    traj_ref: PoseTrajectory3D, traj_est: PoseTrajectory3D, max_diff: float
) -> Tuple[PoseTrajectory3D, PoseTrajectory3D, metrics.APE, metrics.RPE, dict]:
    """Associate, align (SE3, no scale), and compute ATE/RPE + drift metrics."""
    ref_sync, est_sync = sync.associate_trajectories(traj_ref, traj_est, max_diff=max_diff)

    est_aligned = copy.deepcopy(est_sync)
    # Metric VIO -> rigid SE(3) Umeyama (no scale). A near-straight-line flight
    # gives a rank-deficient covariance (Umeyama undefined); fall back to
    # anchoring the start pose, which is also the more honest drift reference.
    align_method = "umeyama_se3"
    try:
        est_aligned.align(ref_sync, correct_scale=False)
    except GeometryException:
        # Collinear (straight-line) trajectory: full Umeyama rotation is
        # rank-deficient. Remove only the 4 unobservable gauge DOF (translation
        # + yaw); see _gauge_align_yaw_translation. This replaces the
        # origin-only fallback, which left the heading gauge uncorrected.
        transform = _gauge_align_yaw_translation(ref_sync, est_sync)
        est_aligned = copy.deepcopy(est_sync)
        est_aligned.transform(transform)
        align_method = "yaw+translation_gauge"

    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref_sync, est_aligned))

    # RPE over 1 m segments (drift rate). Falls back gracefully on trajectories
    # too short to contain a 1 m segment (degenerate / near-stationary bags).
    rpe = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=1.0,
        delta_unit=metrics.Unit.meters,
        all_pairs=False,
    )
    try:
        rpe.process_data((ref_sync, est_aligned))
        rpe_rmse = round(float(rpe.get_statistic(metrics.StatisticsType.rmse)), 5)
        rpe_max = round(float(rpe.get_statistic(metrics.StatisticsType.max)), 5)
    except filters.FilterException:
        rpe = None
        rpe_rmse = rpe_max = None

    path_length = float(ref_sync.path_length)  # metres travelled (ground truth)
    ate_rmse = float(ape.get_statistic(metrics.StatisticsType.rmse))
    ate_max = float(ape.get_statistic(metrics.StatisticsType.max))
    final_err = float(ape.error[-1])

    summary = {
        "align_method": align_method,
        "n_poses_ref": int(traj_ref.num_poses),
        "n_poses_est": int(traj_est.num_poses),
        "n_poses_synced": int(ref_sync.num_poses),
        "duration_s": round(float(ref_sync.timestamps[-1] - ref_sync.timestamps[0]), 3),
        "path_length_m": round(path_length, 3),
        "ate_rmse_m": round(ate_rmse, 4),
        "ate_max_m": round(ate_max, 4),
        "rpe_rmse_m_per_m": rpe_rmse,
        "rpe_max_m_per_m": rpe_max,
        "final_drift_m": round(final_err, 4),
        # Two drift conventions; both reported. Honeywell target: <1.5% over 200 m.
        "drift_pct_ate": round(100.0 * ate_rmse / path_length, 3) if path_length > 0 else None,
        "drift_pct_final": round(100.0 * final_err / path_length, 3) if path_length > 0 else None,
    }
    # KITTI segment drift (alignment-free drift RATE; the spec-faithful metric).
    seg_lengths = [L for L in (5, 10, 15, 20, 25, 30, 40, 50, 75, 100) if L < 0.9 * path_length]
    if seg_lengths:
        kitti = _kitti_segment_drift(ref_sync, est_aligned, seg_lengths)
        summary["kitti_drift_pct_by_len"] = kitti
        summary["kitti_drift_pct_mean"] = (
            round(float(np.mean(list(kitti.values()))), 3) if kitti else None
        )

    return ref_sync, est_aligned, ape, rpe, summary


def _plot_trajectory(ref, est, ape, out_path: Path) -> None:
    """Top-down (XY) + side (XZ) trajectory overlay, VIO coloured by APE."""
    fig = plt.figure(figsize=(13, 6))
    for idx, mode in enumerate((evo_plot.PlotMode.xy, evo_plot.PlotMode.xz), start=1):
        ax = fig.add_subplot(1, 2, idx)
        evo_plot.traj(ax, mode, ref, style="--", color="0.4", label="ground truth")
        evo_plot.traj_colormap(
            ax, est, ape.error, mode,
            min_map=float(np.min(ape.error)), max_map=float(np.max(ape.error)),
        )
        # evo forces set_aspect("equal"); on a near-straight run (Scenario A/B:
        # y/z ~= const) equal aspect squishes the path to a sub-pixel sliver, so
        # the trajectory renders invisible. Force auto aspect + pad a degenerate
        # axis so a flat line stays visible across the plot.
        ax.set_aspect("auto")
        ax.autoscale_view()
        ylo, yhi = ax.get_ylim()
        if yhi - ylo < 0.5:
            mid = 0.5 * (ylo + yhi)
            ax.set_ylim(mid - 0.5, mid + 0.5)
        ax.set_title(f"trajectory ({mode.value})")
        ax.legend(loc="best")
    fig.suptitle("ARGUS VIO vs ground truth (VIO colour = APE, m)")
    fig.savefig(out_path)
    plt.close(fig)


def _plot_error_over_distance(ref_sync, ape, summary, out_path: Path) -> None:
    """APE translation error against distance travelled, with drift target line."""
    dist = ref_sync.distances  # cumulative path length per pose (m)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(dist, ape.error, color="C0", lw=1.6, label="APE (translation)")
    ax.axhline(ape.get_statistic(metrics.StatisticsType.rmse), color="C1", ls="--",
               label=f"RMSE = {summary['ate_rmse_m']:.3f} m")
    # 1.5%-of-distance Honeywell budget envelope.
    ax.plot(dist, 0.015 * dist, color="C3", ls=":", label="1.5% drift budget")
    ax.set_xlabel("distance travelled (m)")
    ax.set_ylabel("absolute position error (m)")
    ax.set_title(
        f"Drift vs distance | path {summary['path_length_m']:.1f} m | "
        f"drift {summary['drift_pct_ate']}% (ATE) / {summary['drift_pct_final']}% (final)"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.savefig(out_path)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="ARGUS VIO evaluation (evo).")
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory")
    parser.add_argument("--run-id", default=None, help="output subdir name under --out-root")
    parser.add_argument("--out-root", default=Path("data/eval"), type=Path)
    parser.add_argument("--gt-topic", default=DEFAULT_GT_TOPIC)
    parser.add_argument("--vio-topic", default=DEFAULT_VIO_TOPIC)
    parser.add_argument("--max-diff", default=0.05, type=float,
                        help="max stamp diff (s) for ref/est association")
    parser.add_argument("--skip-start-m", default=0.0, type=float,
                        help="exclude the first N metres of GT path before "
                             "evaluating (VIO initialization region; the "
                             "estimator is not yet converged there, so it is "
                             "not 'drift'). Standard VIO-benchmark practice.")
    parser.add_argument("--max-dist-m", default=0.0, type=float,
                        help="evaluate only up to N metres of GT path (after any "
                             "--skip-start-m). Use to isolate a single leg of a "
                             "multi-leg shuttle (e.g. Scenario A = first forward "
                             "leg) from one recording. 0 = full trajectory.")
    args = parser.parse_args()

    if not args.bag.exists():
        raise SystemExit(f"[run_eval] bag not found: {args.bag}")
    run_id = args.run_id or args.bag.name
    out_dir = args.out_root / run_id
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[run_eval] bag={args.bag}  run_id={run_id}")
    traj_ref = read_trajectory(args.bag, args.gt_topic)
    traj_est = read_trajectory(args.bag, args.vio_topic)
    print(f"[run_eval] ground truth poses: {traj_ref.num_poses}, VIO poses: {traj_est.num_poses}")

    # Optionally drop the VIO initialization region (first N m of GT path). Before
    # VINS reports "Initialization finish!" its pose is not a converged estimate,
    # so including it measures init transient, not drift. Excluding it is the
    # standard convention; the full-trajectory number is still reported separately.
    if args.skip_start_m > 0.0:
        dists = traj_ref.distances
        over = np.where(dists >= args.skip_start_m)[0]
        if len(over) > 0:
            t0 = float(traj_ref.timestamps[over[0]])
            traj_ref.reduce_to_time_range(t0, float(traj_ref.timestamps[-1]))
            traj_est.reduce_to_time_range(t0, float(traj_est.timestamps[-1]))
            print(f"[run_eval] excluded first {args.skip_start_m:.1f} m (init "
                  f"region) -> ref {traj_ref.num_poses} / est {traj_est.num_poses} poses")

    # Optionally keep only the first N m of GT path (isolate one shuttle leg).
    if args.max_dist_m > 0.0:
        d0 = traj_ref.distances
        base = float(d0[0])
        over = np.where((d0 - base) >= args.max_dist_m)[0]
        if len(over) > 0:
            t1 = float(traj_ref.timestamps[over[0]])
            traj_ref.reduce_to_time_range(float(traj_ref.timestamps[0]), t1)
            traj_est.reduce_to_time_range(float(traj_est.timestamps[0]), t1)
            print(f"[run_eval] kept first {args.max_dist_m:.1f} m -> ref "
                  f"{traj_ref.num_poses} / est {traj_est.num_poses} poses")

    ref_sync, est_aligned, ape, rpe, summary = evaluate(traj_ref, traj_est, args.max_diff)
    summary["skip_start_m"] = args.skip_start_m
    summary["max_dist_m"] = args.max_dist_m

    # Persist trajectories (TUM) + metrics + plots.
    from evo.tools import file_interface
    file_interface.write_tum_trajectory_file(out_dir / "gt.tum", traj_ref)
    file_interface.write_tum_trajectory_file(out_dir / "vio.tum", est_aligned)
    (out_dir / "metrics.json").write_text(json.dumps(summary, indent=2))

    try:
        _plot_trajectory(ref_sync, est_aligned, ape, out_dir / "trajectory.png")
        _plot_error_over_distance(ref_sync, ape, summary, out_dir / "error_over_distance.png")
    except Exception as exc:  # plotting must never lose the metrics
        print(f"[run_eval] WARNING: plotting failed: {exc}")

    print("\n===== ARGUS VIO eval summary =====")
    for key, val in summary.items():
        print(f"  {key:>20}: {val}")
    print(f"  outputs -> {out_dir}")
    print("==================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
