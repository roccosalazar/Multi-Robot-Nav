#!/usr/bin/env python3
"""Compare MRG and Swarm multi-robot SLAM benchmark runs.

The script consumes the outputs produced by ``analyze_run.py`` under
``results/<algorithm>/<run_id>/analysis_pose_graph``.  It keeps the comparison
world-aware, selects one canonical pose-graph source per run using only
completeness/size criteria, writes paper-oriented CSV tables, and creates
trajectory overlays from final per-keyframe aligned positions.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import re
import shutil
import sys
from typing import Iterable

import numpy as np
import pandas as pd


ALGORITHMS = {
    "mrg_multi": "MRG",
    "swarm_multi": "Swarm",
}

ROBOT_IDS = [0, 1, 2]

PRIMARY_METRICS = [
    "ate_m_rmse",
    "ate_m_mean",
    "ate_m_median",
    "ate_m_p95",
    "ate_m_max",
    "ate_m_final",
    "orientation_error_deg_rmse",
    "orientation_error_deg_p95",
    "drift_percent",
    "rpe_dist_1m_trans_rmse_m",
    "rpe_dist_5m_trans_rmse_m",
    "rpe_dist_10m_trans_rmse_m",
    "rpe_dist_1m_rot_rmse_deg",
    "rpe_dist_5m_rot_rmse_deg",
    "rpe_dist_10m_rot_rmse_deg",
    "rpe_kf_1_trans_rmse_m",
    "rpe_kf_5_trans_rmse_m",
    "rpe_kf_10_trans_rmse_m",
    "coverage_fraction",
    "scale_ratio_graph_to_gt",
    "graph_path_length_m",
    "gt_matched_path_length_m",
    "node_count_robot",
]

GRAPH_METRICS = [
    "node_count_total",
    "edge_count_total",
    "graph_odom_edges",
    "graph_intra_loop_closures",
    "graph_inter_loop_closures",
    "graph_valid_inter_robot_loop_closures",
    "graph_invalid_inter_robot_loop_closures",
    "graph_inter_robot_loop_closure_precision",
    "graph_inter_robot_loop_closure_recall",
    "graph_real_inter_robot_loop_opportunities",
    "graph_found_real_inter_robot_loop_opportunities",
    "global_map_consistency_error_m",
    "graph_map_consistency_trans_rmse_m",
    "graph_map_consistency_rot_rmse_deg",
    "graph_inter_robot_loop_measurement_trans_error_rmse_m",
    "graph_inter_robot_loop_measurement_rot_error_rmse_deg",
    "time_rel_sec",
]

LOWER_BETTER_METRICS = {
    "ate_m_rmse",
    "ate_m_mean",
    "ate_m_median",
    "ate_m_p95",
    "ate_m_max",
    "ate_m_final",
    "orientation_error_deg_rmse",
    "orientation_error_deg_p95",
    "drift_percent",
    "rpe_dist_1m_trans_rmse_m",
    "rpe_dist_5m_trans_rmse_m",
    "rpe_dist_10m_trans_rmse_m",
    "rpe_dist_1m_rot_rmse_deg",
    "rpe_dist_5m_rot_rmse_deg",
    "rpe_dist_10m_rot_rmse_deg",
    "rpe_kf_1_trans_rmse_m",
    "rpe_kf_5_trans_rmse_m",
    "rpe_kf_10_trans_rmse_m",
    "global_map_consistency_error_m",
    "graph_map_consistency_trans_rmse_m",
    "graph_map_consistency_rot_rmse_deg",
    "graph_inter_robot_loop_measurement_trans_error_rmse_m",
    "graph_inter_robot_loop_measurement_rot_error_rmse_deg",
}

HIGHER_BETTER_METRICS = {
    "coverage_fraction",
    "graph_inter_robot_loop_closure_precision",
    "graph_inter_robot_loop_closure_recall",
    "graph_found_real_inter_robot_loop_opportunities",
    "node_count_robot",
    "node_count_total",
}


@dataclass(frozen=True)
class RunInfo:
    algorithm_dir: str
    algorithm: str
    run_id: str
    run_dir: Path
    world: str
    world_source: str
    geometry_world: str
    geometry_max_span_m: float
    geometry_max_path_length_m: float
    world_note: str
    has_analysis: bool
    has_point_errors: bool


@dataclass(frozen=True)
class SourceChoice:
    source: str | None
    valid_for_paper: bool
    reason: str
    coverage_min: float
    coverage_mean: float
    node_count_total: float
    edge_count_total: float
    robot_count: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build world-aware benchmark comparison tables and trajectory "
            "overlays for results/mrg_multi and results/swarm_multi."
        )
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("results"),
        help="Root containing mrg_multi/ and swarm_multi/.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/benchmark_comparison"),
        help="Directory where comparison CSVs, plots, and report are written.",
    )
    parser.add_argument(
        "--world",
        action="append",
        default=None,
        help=(
            "World to include. Can be repeated. Defaults to all worlds that "
            "can be inferred."
        ),
    )
    parser.add_argument(
        "--coverage-threshold",
        type=float,
        default=0.90,
        help=(
            "Minimum per-robot coverage required for a source to be included "
            "in paper aggregate tables."
        ),
    )
    parser.add_argument(
        "--large-trajectory-world",
        default="cave",
        help=(
            "World name assigned when GT geometry is much larger than a "
            "warehouse-scale run."
        ),
    )
    parser.add_argument(
        "--large-trajectory-span-threshold",
        type=float,
        default=100.0,
        help="Classify a run as --large-trajectory-world if any robot XY span exceeds this value.",
    )
    parser.add_argument(
        "--large-trajectory-length-threshold",
        type=float,
        default=500.0,
        help="Classify a run as --large-trajectory-world if any robot XY path length exceeds this value.",
    )
    parser.add_argument(
        "--include-incomplete",
        action="store_true",
        help="Include incomplete runs in aggregate tables, flagged as not paper-valid.",
    )
    parser.add_argument(
        "--all-pair-overlays",
        action="store_true",
        help=(
            "Generate trajectory overlays for every MRG/Swarm valid run pair "
            "within each world. By default only the latest valid pair per world "
            "is plotted."
        ),
    )
    parser.add_argument(
        "--mrg-run",
        default=None,
        help="Specific MRG run id to use for overlays.",
    )
    parser.add_argument(
        "--swarm-run",
        default=None,
        help="Specific Swarm run id to use for overlays.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Write CSV/report only.",
    )
    parser.add_argument(
        "--world-override",
        action="append",
        default=[],
        metavar="ALG:RUN=WORLD",
        help=(
            "Override world inference for a run, for example "
            "mrg_multi:20260520_091321=warehouse. Can be repeated."
        ),
    )
    return parser.parse_args()


def parse_world_overrides(items: Iterable[str]) -> dict[tuple[str, str], str]:
    overrides: dict[tuple[str, str], str] = {}
    for item in items:
        if "=" not in item or ":" not in item.split("=", 1)[0]:
            raise SystemExit(
                "--world-override must look like ALG:RUN=WORLD, "
                f"got: {item}"
            )
        left, world = item.split("=", 1)
        algorithm_dir, run_id = left.split(":", 1)
        overrides[(algorithm_dir, run_id)] = world
    return overrides


def first_gt_frame_id(run_dir: Path) -> tuple[str, str] | None:
    frames: list[str] = []
    for gt_path in sorted((run_dir / "ground_truth").glob("ground_truth_r*.csv")):
        try:
            with gt_path.open(newline="") as handle:
                reader = csv.DictReader(handle)
                row = next(reader, None)
        except (OSError, csv.Error):
            continue
        if row is None:
            continue
        frame_id = (row.get("frame_id") or "").strip()
        if frame_id:
            frames.append(frame_id)
    if not frames:
        return None
    unique = sorted(set(frames))
    return unique[0], "ground_truth.frame_id" if len(unique) == 1 else "mixed_ground_truth.frame_id"


def frame_id_from_logs(run_dir: Path) -> tuple[str, str] | None:
    pattern = re.compile(r"SlamPosePublisher started:.*frame_id='([^']+)'")
    found: list[str] = []
    for log_path in sorted(run_dir.glob("*.log")):
        try:
            for line in log_path.read_text(errors="replace").splitlines():
                match = pattern.search(line)
                if match:
                    found.append(match.group(1).strip())
        except OSError:
            continue
    found = [value for value in found if value and value.lower() != "auto"]
    if not found:
        return None
    unique = sorted(set(found))
    return unique[0], "slam_pose_publisher.log" if len(unique) == 1 else "mixed_slam_pose_publisher.log"


def ground_truth_geometry_world(
    run_dir: Path,
    large_world: str,
    span_threshold_m: float,
    length_threshold_m: float,
) -> tuple[str, float, float, str]:
    max_span = 0.0
    max_path_length = 0.0
    loaded = False
    for gt_path in sorted((run_dir / "ground_truth").glob("ground_truth_r*.csv")):
        try:
            df = pd.read_csv(gt_path, usecols=["position_x", "position_y"])
        except (OSError, ValueError, pd.errors.EmptyDataError):
            continue
        if df.empty:
            continue
        loaded = True
        x = pd.to_numeric(df["position_x"], errors="coerce").to_numpy()
        y = pd.to_numeric(df["position_y"], errors="coerce").to_numpy()
        finite = np.isfinite(x) & np.isfinite(y)
        if not finite.any():
            continue
        x = x[finite]
        y = y[finite]
        span = max(float(np.nanmax(x) - np.nanmin(x)), float(np.nanmax(y) - np.nanmin(y)))
        step = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)
        path_length = float(np.nansum(step))
        max_span = max(max_span, span)
        max_path_length = max(max_path_length, path_length)

    if not loaded:
        return "unknown", math.nan, math.nan, "no_ground_truth"
    if max_span >= span_threshold_m or max_path_length >= length_threshold_m:
        return large_world, max_span, max_path_length, "large_trajectory_signature"
    return "small", max_span, max_path_length, "small_trajectory_signature"


def infer_world(
    algorithm_dir: str,
    run_id: str,
    run_dir: Path,
    overrides: dict[tuple[str, str], str],
    large_world: str,
    span_threshold_m: float,
    length_threshold_m: float,
) -> tuple[str, str, str, float, float, str]:
    geometry_world, max_span, max_path_length, geometry_reason = ground_truth_geometry_world(
        run_dir,
        large_world,
        span_threshold_m,
        length_threshold_m,
    )
    override = overrides.get((algorithm_dir, run_id))
    if override:
        return override, "override", geometry_world, max_span, max_path_length, geometry_reason

    gt_world = first_gt_frame_id(run_dir)
    log_world = frame_id_from_logs(run_dir)

    if gt_world and gt_world[0].lower() != "auto":
        note = geometry_reason
        if geometry_world == large_world and gt_world[0] != large_world:
            note = f"gt_geometry_conflict:{geometry_reason}"
        return gt_world[0], gt_world[1], geometry_world, max_span, max_path_length, note
    if geometry_world == large_world:
        note = geometry_reason
        if log_world and log_world[0] != large_world:
            note = f"{geometry_reason};ignored_log_world={log_world[0]}"
        return large_world, "ground_truth.geometry", geometry_world, max_span, max_path_length, note
    if log_world:
        return log_world[0], log_world[1], geometry_world, max_span, max_path_length, geometry_reason
    if gt_world:
        return gt_world[0], gt_world[1], geometry_world, max_span, max_path_length, geometry_reason
    return "unknown", "not_found", geometry_world, max_span, max_path_length, geometry_reason


def discover_runs(
    results_root: Path,
    overrides: dict[tuple[str, str], str],
    large_world: str,
    span_threshold_m: float,
    length_threshold_m: float,
) -> list[RunInfo]:
    runs: list[RunInfo] = []
    for algorithm_dir, algorithm in ALGORITHMS.items():
        base = results_root / algorithm_dir
        if not base.exists():
            continue
        for run_dir in sorted(path for path in base.iterdir() if path.is_dir()):
            run_id = run_dir.name
            (
                world,
                world_source,
                geometry_world,
                geometry_max_span_m,
                geometry_max_path_length_m,
                world_note,
            ) = infer_world(
                algorithm_dir,
                run_id,
                run_dir,
                overrides,
                large_world,
                span_threshold_m,
                length_threshold_m,
            )
            analysis_dir = run_dir / "analysis_pose_graph"
            runs.append(
                RunInfo(
                    algorithm_dir=algorithm_dir,
                    algorithm=algorithm,
                    run_id=run_id,
                    run_dir=run_dir,
                    world=world,
                    world_source=world_source,
                    geometry_world=geometry_world,
                    geometry_max_span_m=geometry_max_span_m,
                    geometry_max_path_length_m=geometry_max_path_length_m,
                    world_note=world_note,
                    has_analysis=(analysis_dir / "summary_metrics.csv").exists(),
                    has_point_errors=(analysis_dir / "point_errors").exists(),
                )
            )
    return runs


def safe_float(value: object) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out


def choose_source(summary: pd.DataFrame, coverage_threshold: float) -> SourceChoice:
    if summary.empty or "source" not in summary:
        return SourceChoice(None, False, "empty_summary", math.nan, math.nan, 0.0, 0.0, 0)

    candidates: list[dict[str, object]] = []
    for source, group in summary.groupby("source", sort=True):
        robot_count = group["robot_id"].nunique() if "robot_id" in group else len(group)
        coverage = pd.to_numeric(group.get("coverage_fraction"), errors="coerce")
        coverage_min = float(coverage.min(skipna=True)) if coverage.notna().any() else 0.0
        coverage_mean = float(coverage.mean(skipna=True)) if coverage.notna().any() else 0.0
        node_count_total = float(
            pd.to_numeric(group.get("node_count_total"), errors="coerce").max(skipna=True)
        )
        edge_count_total = float(
            pd.to_numeric(group.get("edge_count_total"), errors="coerce").max(skipna=True)
        )
        snapshots = float(
            pd.to_numeric(group.get("snapshots_analyzed"), errors="coerce").max(skipna=True)
        )
        paper_valid = (
            robot_count >= len(ROBOT_IDS)
            and coverage_min >= coverage_threshold
            and np.isfinite(node_count_total)
            and node_count_total > 0
        )
        candidates.append(
            {
                "source": source,
                "robot_count": robot_count,
                "coverage_min": coverage_min,
                "coverage_mean": coverage_mean,
                "node_count_total": node_count_total,
                "edge_count_total": edge_count_total,
                "snapshots_analyzed": snapshots,
                "paper_valid": paper_valid,
            }
        )

    valid = [item for item in candidates if item["paper_valid"]]
    pool = valid if valid else candidates

    def sort_key(item: dict[str, object]) -> tuple[float, float, float, float, str]:
        return (
            safe_float(item["coverage_min"]),
            safe_float(item["node_count_total"]),
            safe_float(item["edge_count_total"]),
            safe_float(item["snapshots_analyzed"]),
            str(item["source"]),
        )

    selected = max(pool, key=sort_key)
    valid_for_paper = bool(selected["paper_valid"])
    reason = "coverage_threshold_met" if valid_for_paper else "best_available_but_incomplete"
    return SourceChoice(
        source=str(selected["source"]),
        valid_for_paper=valid_for_paper,
        reason=reason,
        coverage_min=safe_float(selected["coverage_min"]),
        coverage_mean=safe_float(selected["coverage_mean"]),
        node_count_total=safe_float(selected["node_count_total"]),
        edge_count_total=safe_float(selected["edge_count_total"]),
        robot_count=int(selected["robot_count"]),
    )


def load_summary(run: RunInfo) -> pd.DataFrame | None:
    path = run.run_dir / "analysis_pose_graph" / "summary_metrics.csv"
    if not path.exists():
        return None
    return pd.read_csv(path)


def build_inventory(
    runs: list[RunInfo],
    coverage_threshold: float,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    inventory_rows: list[dict[str, object]] = []
    canonical_rows: list[pd.DataFrame] = []

    for run in runs:
        choice = SourceChoice(None, False, "missing_analysis", math.nan, math.nan, 0.0, 0.0, 0)
        summary = load_summary(run) if run.has_analysis else None
        if summary is not None:
            choice = choose_source(summary, coverage_threshold)
            if choice.source is not None:
                selected_rows = summary.loc[summary["source"] == choice.source].copy()
                selected_rows.insert(0, "algorithm", run.algorithm)
                selected_rows.insert(0, "algorithm_dir", run.algorithm_dir)
                selected_rows.insert(0, "run_id", run.run_id)
                selected_rows.insert(0, "world", run.world)
                selected_rows["world_source"] = run.world_source
                selected_rows["canonical_source"] = choice.source
                selected_rows["valid_for_paper"] = choice.valid_for_paper
                selected_rows["source_selection_reason"] = choice.reason
                canonical_rows.append(selected_rows)

        inventory_rows.append(
            {
                "algorithm_dir": run.algorithm_dir,
                "algorithm": run.algorithm,
                "run_id": run.run_id,
                "run_dir": str(run.run_dir),
                "world": run.world,
                "world_source": run.world_source,
                "geometry_world": run.geometry_world,
                "geometry_max_span_m": run.geometry_max_span_m,
                "geometry_max_path_length_m": run.geometry_max_path_length_m,
                "world_note": run.world_note,
                "has_analysis": run.has_analysis,
                "has_point_errors": run.has_point_errors,
                "canonical_source": choice.source,
                "valid_for_paper": choice.valid_for_paper,
                "source_selection_reason": choice.reason,
                "coverage_min": choice.coverage_min,
                "coverage_mean": choice.coverage_mean,
                "node_count_total": choice.node_count_total,
                "edge_count_total": choice.edge_count_total,
                "robot_count": choice.robot_count,
            }
        )

    inventory = pd.DataFrame(inventory_rows)
    canonical = pd.concat(canonical_rows, ignore_index=True) if canonical_rows else pd.DataFrame()
    return inventory, canonical


def selected_worlds(canonical: pd.DataFrame, args: argparse.Namespace) -> list[str]:
    worlds = args.world
    if worlds:
        return sorted(set(worlds))
    if canonical.empty or "world" not in canonical:
        return []
    return sorted(
        world
        for world in canonical["world"].dropna().unique()
        if world and str(world).lower() not in {"auto", "unknown"}
    )


def metric_columns(df: pd.DataFrame, requested: list[str]) -> list[str]:
    return [col for col in requested if col in df.columns]


def weighted_mean(values: pd.Series, weights: pd.Series) -> float:
    values = pd.to_numeric(values, errors="coerce")
    weights = pd.to_numeric(weights, errors="coerce").fillna(0)
    mask = values.notna() & weights.gt(0)
    if not mask.any():
        return math.nan
    return float(np.average(values[mask], weights=weights[mask]))


def build_run_level_metrics(canonical: pd.DataFrame) -> pd.DataFrame:
    if canonical.empty:
        return pd.DataFrame()

    rows: list[dict[str, object]] = []
    primary_cols = metric_columns(canonical, PRIMARY_METRICS)
    graph_cols = metric_columns(canonical, GRAPH_METRICS)
    group_cols = [
        "world",
        "algorithm_dir",
        "algorithm",
        "run_id",
        "canonical_source",
        "valid_for_paper",
    ]
    for keys, group in canonical.groupby(group_cols, dropna=False, sort=True):
        row = dict(zip(group_cols, keys))
        row["robot_count"] = group["robot_id"].nunique()
        row["coverage_min"] = pd.to_numeric(group.get("coverage_fraction"), errors="coerce").min()
        row["coverage_mean"] = pd.to_numeric(group.get("coverage_fraction"), errors="coerce").mean()
        row["node_count_total"] = pd.to_numeric(group.get("node_count_total"), errors="coerce").max()
        row["edge_count_total"] = pd.to_numeric(group.get("edge_count_total"), errors="coerce").max()
        weights = group.get("node_count_robot", pd.Series(1.0, index=group.index))
        for col in primary_cols:
            row[f"{col}_macro_mean"] = pd.to_numeric(group[col], errors="coerce").mean()
            row[f"{col}_weighted_mean"] = weighted_mean(group[col], weights)
            row[f"{col}_max_robot"] = pd.to_numeric(group[col], errors="coerce").max()
            row[f"{col}_min_robot"] = pd.to_numeric(group[col], errors="coerce").min()
        for col in graph_cols:
            row[col] = pd.to_numeric(group[col], errors="coerce").max()
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_group(df: pd.DataFrame, group_cols: list[str], metric_cols: list[str]) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for keys, group in df.groupby(group_cols, dropna=False, sort=True):
        row = dict(zip(group_cols, keys if isinstance(keys, tuple) else (keys,)))
        row["n_runs"] = group["run_id"].nunique() if "run_id" in group else len(group)
        for col in metric_cols:
            values = pd.to_numeric(group[col], errors="coerce").dropna()
            if values.empty:
                continue
            row[f"{col}_mean"] = float(values.mean())
            row[f"{col}_std"] = float(values.std(ddof=1)) if len(values) > 1 else 0.0
            row[f"{col}_median"] = float(values.median())
            row[f"{col}_min"] = float(values.min())
            row[f"{col}_max"] = float(values.max())
            row[f"{col}_ci95"] = (
                1.96 * float(values.std(ddof=1)) / math.sqrt(len(values))
                if len(values) > 1
                else 0.0
            )
        rows.append(row)
    return pd.DataFrame(rows)


def build_pairwise_deltas(summary: pd.DataFrame, group_cols: list[str]) -> pd.DataFrame:
    if summary.empty:
        return pd.DataFrame()
    metric_mean_cols = [col for col in summary.columns if col.endswith("_mean")]
    rows: list[dict[str, object]] = []
    scope = "robot" if "robot_id" in group_cols else "run"
    for keys, group in summary.groupby(group_cols, dropna=False, sort=True):
        if set(group["algorithm"]) < {"MRG", "Swarm"}:
            continue
        mrg = group.loc[group["algorithm"] == "MRG"].iloc[0]
        swarm = group.loc[group["algorithm"] == "Swarm"].iloc[0]
        base = dict(zip(group_cols, keys if isinstance(keys, tuple) else (keys,)))
        for mean_col in metric_mean_cols:
            metric = mean_col[: -len("_mean")]
            mrg_value = safe_float(mrg.get(mean_col))
            swarm_value = safe_float(swarm.get(mean_col))
            if not np.isfinite(mrg_value) or not np.isfinite(swarm_value):
                continue
            delta = mrg_value - swarm_value
            ratio = mrg_value / swarm_value if abs(swarm_value) > 1e-12 else math.nan
            better = metric_direction(metric)
            row = {
                **base,
                "scope": scope,
                "metric": metric,
                "better": better,
                "mrg_mean": mrg_value,
                "swarm_mean": swarm_value,
                "mrg_minus_swarm": delta,
                "mrg_over_swarm": ratio,
            }
            if better == "lower":
                row["relative_improvement_mrg_vs_swarm_percent"] = (
                    (swarm_value - mrg_value) / abs(swarm_value) * 100.0
                    if abs(swarm_value) > 1e-12
                    else math.nan
                )
            elif better == "higher":
                row["relative_improvement_mrg_vs_swarm_percent"] = (
                    (mrg_value - swarm_value) / abs(swarm_value) * 100.0
                    if abs(swarm_value) > 1e-12
                    else math.nan
                )
            else:
                row["relative_improvement_mrg_vs_swarm_percent"] = math.nan
            rows.append(row)
    return pd.DataFrame(rows)


def metric_direction(metric: str) -> str:
    base = metric
    for suffix in ["_macro_mean", "_weighted_mean", "_max_robot", "_min_robot"]:
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    if base in LOWER_BETTER_METRICS:
        return "lower"
    if base in HIGHER_BETTER_METRICS:
        return "higher"
    return "context"


def metric_definitions() -> pd.DataFrame:
    rows = [
        ("ate_m_rmse", "m", "robot trajectory", "lower", "Absolute trajectory error RMSE after graph-to-GT alignment."),
        ("ate_m_p95", "m", "robot trajectory", "lower", "95th percentile absolute trajectory error."),
        ("ate_m_max", "m", "robot trajectory", "lower", "Worst final per-keyframe absolute trajectory error."),
        ("ate_m_final", "m", "robot trajectory", "lower", "Final keyframe absolute trajectory error."),
        ("orientation_error_deg_rmse", "deg", "robot trajectory", "lower", "Orientation error RMSE after alignment."),
        ("drift_percent", "%", "robot trajectory", "lower", "Final translational drift normalized by matched GT path length."),
        ("rpe_dist_5m_trans_rmse_m", "m", "local odometry", "lower", "Relative pose translational error over approximately 5 m intervals."),
        ("rpe_dist_5m_rot_rmse_deg", "deg", "local odometry", "lower", "Relative pose rotational error over approximately 5 m intervals."),
        ("coverage_fraction", "fraction", "robot trajectory", "higher", "Fraction of the recorded GT path covered by matched graph keyframes."),
        ("scale_ratio_graph_to_gt", "ratio", "robot trajectory", "near_1", "Graph path length divided by matched GT path length."),
        ("node_count_total", "count", "graph", "context", "Total graph nodes in the selected source."),
        ("edge_count_total", "count", "graph", "context", "Total graph edges in the selected source."),
        ("graph_inter_loop_closures", "count", "loop closures", "context", "Accepted inter-robot loop closures in the full graph."),
        ("graph_inter_robot_loop_closure_precision", "fraction", "loop closures", "higher", "Accepted inter-robot loop closures validated against GT relative pose."),
        ("graph_inter_robot_loop_closure_recall", "fraction", "loop closures", "higher", "Accepted valid inter-robot closures divided by GT proximity opportunities."),
        ("global_map_consistency_error_m", "m", "graph consistency", "lower", "Translational RMSE between optimized inter-robot relative poses and loop measurements."),
        ("graph_map_consistency_rot_rmse_deg", "deg", "graph consistency", "lower", "Rotational RMSE between optimized inter-robot relative poses and loop measurements."),
    ]
    return pd.DataFrame(rows, columns=["metric", "unit", "scope", "better", "definition"])


def point_errors_path(run: RunInfo, source: str, robot_id: int) -> Path:
    return (
        run.run_dir
        / "analysis_pose_graph"
        / "point_errors"
        / f"{source}_r{robot_id}_final_point_errors.csv"
    )


def downsample(df: pd.DataFrame, max_points: int = 2000) -> pd.DataFrame:
    if len(df) <= max_points:
        return df
    idx = np.linspace(0, len(df) - 1, max_points).round().astype(int)
    return df.iloc[idx]


def trajectory_extent(frames: list[pd.DataFrame]) -> tuple[float, float, float, float]:
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for df in frames:
        for x_col, y_col in [
            ("gt_x", "gt_y"),
            ("aligned_graph_x", "aligned_graph_y"),
        ]:
            if x_col in df and y_col in df:
                xs.append(pd.to_numeric(df[x_col], errors="coerce").to_numpy())
                ys.append(pd.to_numeric(df[y_col], errors="coerce").to_numpy())
    if not xs:
        return -1.0, 1.0, -1.0, 1.0
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    x = x[np.isfinite(x)]
    y = y[np.isfinite(y)]
    if len(x) == 0 or len(y) == 0:
        return -1.0, 1.0, -1.0, 1.0
    pad = max(1.0, 0.05 * max(float(x.max() - x.min()), float(y.max() - y.min())))
    return float(x.min() - pad), float(x.max() + pad), float(y.min() - pad), float(y.max() + pad)


def plot_trajectory_overlay(
    mrg_run: RunInfo,
    swarm_run: RunInfo,
    mrg_source: str,
    swarm_source: str,
    output_dir: Path,
) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir = output_dir / "plots" / "trajectory_overlay"
    plot_dir.mkdir(parents=True, exist_ok=True)

    loaded: dict[tuple[str, int], pd.DataFrame] = {}
    all_frames: list[pd.DataFrame] = []
    for label, run, source in [
        ("MRG", mrg_run, mrg_source),
        ("Swarm", swarm_run, swarm_source),
    ]:
        for robot_id in ROBOT_IDS:
            path = point_errors_path(run, source, robot_id)
            if not path.exists():
                continue
            df = pd.read_csv(path)
            df = downsample(df)
            loaded[(label, robot_id)] = df
            all_frames.append(df)

    if not loaded:
        return []

    xmin, xmax, ymin, ymax = trajectory_extent(all_frames)
    robot_colors = {0: "#31688e", 1: "#35b779", 2: "#fdae61"}
    algo_styles = {
        "MRG": {"color": "#2b5c9f", "linestyle": "-", "linewidth": 2.0},
        "Swarm": {"color": "#b83232", "linestyle": "--", "linewidth": 2.0},
    }

    out_paths: list[Path] = []
    fig, axes = plt.subplots(1, len(ROBOT_IDS), figsize=(16, 5.4), squeeze=False)
    for axis, robot_id in zip(axes[0], ROBOT_IDS):
        for label in ["MRG", "Swarm"]:
            df = loaded.get((label, robot_id))
            if df is None or df.empty:
                continue
            if label == "MRG":
                axis.plot(df["gt_x"], df["gt_y"], color="0.2", linewidth=1.2, alpha=0.75, label="GT")
            style = algo_styles[label]
            axis.plot(
                df["aligned_graph_x"],
                df["aligned_graph_y"],
                label=label,
                **style,
            )
            axis.scatter(
                [df["aligned_graph_x"].iloc[0]],
                [df["aligned_graph_y"].iloc[0]],
                color=style["color"],
                s=24,
                marker="o",
                zorder=4,
            )
            axis.scatter(
                [df["aligned_graph_x"].iloc[-1]],
                [df["aligned_graph_y"].iloc[-1]],
                color=style["color"],
                s=32,
                marker="x",
                zorder=4,
            )
        axis.set_title(f"r{robot_id}")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_xlim(xmin, xmax)
        axis.set_ylim(ymin, ymax)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=8)
    fig.suptitle(
        (
            f"{mrg_run.world}: final aligned trajectories "
            f"MRG {mrg_run.run_id}/{mrg_source} vs "
            f"Swarm {swarm_run.run_id}/{swarm_source}"
        ),
        fontsize=12,
    )
    fig.tight_layout()
    out = (
        plot_dir
        / f"trajectory_overlay_{mrg_run.world}_mrg_{mrg_run.run_id}_swarm_{swarm_run.run_id}.png"
    )
    fig.savefig(out, dpi=180)
    plt.close(fig)
    out_paths.append(out)

    fig, axis = plt.subplots(figsize=(8, 8))
    for robot_id in ROBOT_IDS:
        color = robot_colors[robot_id]
        gt_drawn = False
        for label, linestyle in [("MRG", "-"), ("Swarm", "--")]:
            df = loaded.get((label, robot_id))
            if df is None or df.empty:
                continue
            if not gt_drawn:
                axis.plot(df["gt_x"], df["gt_y"], color=color, linewidth=1.0, alpha=0.25)
                gt_drawn = True
            axis.plot(
                df["aligned_graph_x"],
                df["aligned_graph_y"],
                color=color,
                linestyle=linestyle,
                linewidth=2.0,
                label=f"{label} r{robot_id}",
            )
    axis.set_title(f"{mrg_run.world}: combined final trajectories")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_xlim(xmin, xmax)
    axis.set_ylim(ymin, ymax)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize=8, ncol=2)
    fig.tight_layout()
    out = (
        plot_dir
        / f"trajectory_overlay_combined_{mrg_run.world}_mrg_{mrg_run.run_id}_swarm_{swarm_run.run_id}.png"
    )
    fig.savefig(out, dpi=180)
    plt.close(fig)
    out_paths.append(out)
    return out_paths


def compact_bar_value(value: float) -> str:
    abs_value = abs(value)
    if abs_value >= 100:
        return f"{value:.0f}"
    if abs_value >= 10:
        return f"{value:.2f}"
    if abs_value >= 0.01:
        return f"{value:.3f}"
    if abs_value > 0:
        return f"{value:.1e}"
    return "0"


def plot_metric_bars(canonical: pd.DataFrame, output_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    paths: list[Path] = []
    metrics = [
        ("ate_m_rmse", "ATE RMSE [m]", "lower"),
        ("rpe_dist_5m_trans_rmse_m", "RPE 5 m trans RMSE [m]", "lower"),
        ("global_map_consistency_error_m", "Global map consistency [m]", "lower"),
        ("graph_inter_robot_loop_closure_recall", "Inter-robot LC recall", "higher"),
    ]
    data = canonical.loc[canonical["valid_for_paper"]].copy()
    if data.empty:
        return paths
    for metric, ylabel, _ in metrics:
        if metric not in data.columns:
            continue
        grouped = (
            data.groupby(["world", "algorithm", "robot_id"], as_index=False)[metric]
            .mean(numeric_only=True)
            .dropna(subset=[metric])
        )
        if grouped.empty:
            continue
        worlds = sorted(grouped["world"].unique())
        for world in worlds:
            subset = grouped.loc[grouped["world"] == world]
            finite_values = pd.to_numeric(subset[metric], errors="coerce")
            finite_values = finite_values[np.isfinite(finite_values)]
            positive_values = finite_values.loc[finite_values > 0]
            show_value_labels = metric != "graph_inter_robot_loop_closure_recall"
            use_log_scale = (
                len(positive_values) > 1
                and float(positive_values.max() / positive_values.min()) >= 50.0
            )
            fig, axis = plt.subplots(figsize=(9, 5.2))
            robots = sorted(subset["robot_id"].unique())
            x = np.arange(len(robots))
            width = 0.36
            for offset, algorithm in [(-width / 2, "MRG"), (width / 2, "Swarm")]:
                values = []
                for robot_id in robots:
                    row = subset.loc[
                        (subset["algorithm"] == algorithm) & (subset["robot_id"] == robot_id)
                    ]
                    values.append(float(row[metric].iloc[0]) if not row.empty else math.nan)
                bars = axis.bar(x + offset, values, width=width, label=algorithm)
                for bar, value in zip(bars, values):
                    if not show_value_labels:
                        continue
                    if not np.isfinite(value):
                        continue
                    if use_log_scale and value <= 0:
                        continue
                    label_y = value * 1.10 if use_log_scale else value + max(float(finite_values.max()), 1.0) * 0.02
                    axis.text(
                        bar.get_x() + bar.get_width() / 2.0,
                        label_y,
                        compact_bar_value(value),
                        ha="center",
                        va="bottom",
                        fontsize=8,
                    )
            axis.set_xticks(x)
            axis.set_xticklabels([f"r{int(robot)}" for robot in robots])
            if use_log_scale:
                axis.set_yscale("log")
                axis.set_ylim(float(positive_values.min()) / 2.0, float(positive_values.max()) * 2.2)
                axis.set_ylabel(f"{ylabel} (log scale)")
            else:
                if not finite_values.empty and float(finite_values.max()) > 0:
                    axis.set_ylim(0.0, float(finite_values.max()) * 1.18)
                axis.set_ylabel(ylabel)
            axis.set_title(f"{world}: {metric}")
            axis.grid(True, axis="y", which="both", alpha=0.25)
            axis.legend()
            fig.subplots_adjust(left=0.12, right=0.98, top=0.86, bottom=0.14)
            out = plot_dir / f"{metric}_{world}_by_robot.png"
            fig.savefig(out, dpi=180)
            plt.close(fig)
            paths.append(out)
    return paths


def format_markdown_value(value: object, float_digits: int = 4) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        if not np.isfinite(value):
            return ""
        return f"{value:.{float_digits}f}"
    if isinstance(value, (np.floating,)):
        value = float(value)
        if not np.isfinite(value):
            return ""
        return f"{value:.{float_digits}f}"
    if isinstance(value, (np.integer,)):
        return str(int(value))
    text = str(value)
    return text.replace("|", "\\|")


def markdown_table(df: pd.DataFrame, float_digits: int = 4) -> str:
    if df.empty:
        return "_No rows._"
    headers = list(df.columns)
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for _, row in df.iterrows():
        values = [format_markdown_value(row[col], float_digits) for col in headers]
        lines.append("| " + " | ".join(values) + " |")
    return "\n".join(lines)


def run_lookup(inventory: pd.DataFrame) -> dict[tuple[str, str], RunInfo]:
    out: dict[tuple[str, str], RunInfo] = {}
    for row in inventory.itertuples(index=False):
        out[(row.algorithm_dir, row.run_id)] = RunInfo(
            algorithm_dir=row.algorithm_dir,
            algorithm=row.algorithm,
            run_id=row.run_id,
            run_dir=Path(row.run_dir),
            world=row.world,
            world_source=row.world_source,
            geometry_world=row.geometry_world,
            geometry_max_span_m=safe_float(row.geometry_max_span_m),
            geometry_max_path_length_m=safe_float(row.geometry_max_path_length_m),
            world_note=row.world_note,
            has_analysis=bool(row.has_analysis),
            has_point_errors=bool(row.has_point_errors),
        )
    return out


def overlay_pairs(
    inventory: pd.DataFrame,
    worlds: list[str],
    args: argparse.Namespace,
) -> list[tuple[RunInfo, RunInfo, str, str]]:
    lookup = run_lookup(inventory)
    valid = inventory.loc[
        inventory["has_point_errors"]
        & inventory["has_analysis"]
        & inventory["canonical_source"].notna()
        & (inventory["valid_for_paper"] | args.include_incomplete)
    ].copy()
    if args.mrg_run:
        valid = valid.loc[
            (valid["algorithm_dir"] != "mrg_multi") | (valid["run_id"] == args.mrg_run)
        ]
    if args.swarm_run:
        valid = valid.loc[
            (valid["algorithm_dir"] != "swarm_multi") | (valid["run_id"] == args.swarm_run)
        ]

    pairs: list[tuple[RunInfo, RunInfo, str, str]] = []
    for world in worlds:
        mrg = valid.loc[(valid["world"] == world) & (valid["algorithm_dir"] == "mrg_multi")]
        swarm = valid.loc[(valid["world"] == world) & (valid["algorithm_dir"] == "swarm_multi")]
        if mrg.empty or swarm.empty:
            continue
        if args.all_pair_overlays or args.mrg_run or args.swarm_run:
            for _, mrg_row in mrg.iterrows():
                for _, swarm_row in swarm.iterrows():
                    pairs.append(
                        (
                            lookup[("mrg_multi", mrg_row["run_id"])],
                            lookup[("swarm_multi", swarm_row["run_id"])],
                            str(mrg_row["canonical_source"]),
                            str(swarm_row["canonical_source"]),
                        )
                    )
        else:
            mrg_row = mrg.sort_values("run_id").iloc[-1]
            swarm_row = swarm.sort_values("run_id").iloc[-1]
            pairs.append(
                (
                    lookup[("mrg_multi", mrg_row["run_id"])],
                    lookup[("swarm_multi", swarm_row["run_id"])],
                    str(mrg_row["canonical_source"]),
                    str(swarm_row["canonical_source"]),
                )
            )
    return pairs


def write_report(
    output_dir: Path,
    inventory: pd.DataFrame,
    canonical: pd.DataFrame,
    run_level: pd.DataFrame,
    aggregate_summary: pd.DataFrame,
    pairwise: pd.DataFrame,
    plot_paths: list[Path],
    coverage_threshold: float,
) -> None:
    lines: list[str] = []
    lines.append("# MRG vs Swarm Multi-Robot SLAM Benchmark")
    lines.append("")
    lines.append("## Protocol")
    lines.append("")
    lines.append(f"- Canonical source selection requires per-robot coverage >= {coverage_threshold:.2f}.")
    lines.append("- Ties are resolved by graph size, not by error metrics.")
    lines.append("- Worlds are inferred from explicit GT frame ids, with log fallback for `auto` runs.")
    lines.append("- Paper aggregate tables use `valid_for_paper=true` unless incomplete runs are explicitly included.")
    lines.append("")

    if not inventory.empty:
        lines.append("## Run Inventory")
        lines.append("")
        display_cols = [
            "algorithm",
            "run_id",
            "world",
            "world_source",
            "geometry_max_span_m",
            "geometry_max_path_length_m",
            "world_note",
            "canonical_source",
            "valid_for_paper",
            "coverage_min",
            "node_count_total",
        ]
        lines.append(markdown_table(inventory[display_cols]))
        lines.append("")

    if not canonical.empty:
        valid_worlds = []
        valid = canonical.loc[canonical["valid_for_paper"]]
        for world, group in valid.groupby("world", sort=True):
            if set(group["algorithm"]) >= {"MRG", "Swarm"}:
                valid_worlds.append(world)
        missing_worlds = []
        for world in sorted(valid["world"].dropna().unique()):
            algs = set(valid.loc[valid["world"] == world, "algorithm"])
            if algs < {"MRG", "Swarm"}:
                missing_worlds.append(f"{world}: missing {sorted({'MRG', 'Swarm'} - algs)}")
        lines.append("## World Matching")
        lines.append("")
        lines.append(f"- Worlds with paper-valid paired data: {', '.join(valid_worlds) if valid_worlds else 'none'}.")
        lines.append(f"- Worlds currently lacking both algorithms: {', '.join(missing_worlds) if missing_worlds else 'none'}.")
        lines.append("")

    if not run_level.empty:
        lines.append("## Run-Level Headline Metrics")
        lines.append("")
        headline_cols = [
            "world",
            "algorithm",
            "run_id",
            "canonical_source",
            "valid_for_paper",
            "coverage_min",
            "ate_m_rmse_weighted_mean",
            "ate_m_rmse_max_robot",
            "rpe_dist_5m_trans_rmse_m_weighted_mean",
            "global_map_consistency_error_m",
            "graph_inter_robot_loop_closure_precision",
            "graph_inter_robot_loop_closure_recall",
        ]
        headline_cols = [col for col in headline_cols if col in run_level.columns]
        lines.append(markdown_table(run_level[headline_cols]))
        lines.append("")

    if not aggregate_summary.empty:
        lines.append("## Aggregate Summary")
        lines.append("")
        cols = [
            "world",
            "algorithm",
            "n_runs",
            "ate_m_rmse_weighted_mean_mean",
            "ate_m_rmse_weighted_mean_std",
            "rpe_dist_5m_trans_rmse_m_weighted_mean_mean",
            "global_map_consistency_error_m_mean",
            "graph_inter_robot_loop_closure_recall_mean",
        ]
        cols = [col for col in cols if col in aggregate_summary.columns]
        lines.append(markdown_table(aggregate_summary[cols]))
        lines.append("")

    if not pairwise.empty:
        lines.append("## Pairwise MRG Minus Swarm")
        lines.append("")
        wanted = {
            "ate_m_rmse_weighted_mean",
            "ate_m_rmse_max_robot",
            "rpe_dist_5m_trans_rmse_m_weighted_mean",
            "global_map_consistency_error_m",
            "graph_inter_robot_loop_closure_recall",
        }
        subset = pairwise.loc[pairwise["metric"].isin(wanted)].copy()
        if "scope" in subset.columns:
            subset = subset.loc[subset["scope"] == "run"]
        if not subset.empty:
            cols = [
                "world",
                "scope",
                "metric",
                "better",
                "mrg_mean",
                "swarm_mean",
                "mrg_minus_swarm",
                "relative_improvement_mrg_vs_swarm_percent",
            ]
            lines.append(markdown_table(subset[cols]))
            lines.append("")

    if plot_paths:
        lines.append("## Plots")
        lines.append("")
        for path in plot_paths:
            rel = path.relative_to(output_dir)
            lines.append(f"- `{rel}`")
        lines.append("")

    lines.append("## Files")
    lines.append("")
    for name in [
        "run_inventory.csv",
        "canonical_robot_metrics.csv",
        "canonical_run_metrics.csv",
        "robot_metric_summary.csv",
        "run_metric_summary.csv",
        "pairwise_metric_deltas.csv",
        "metric_definitions.csv",
    ]:
        lines.append(f"- `{name}`")
    lines.append("")
    (output_dir / "report.md").write_text("\n".join(lines))


def prepare_output_dir(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for name in [
        "run_inventory.csv",
        "canonical_robot_metrics_all_selected.csv",
        "canonical_robot_metrics.csv",
        "canonical_run_metrics.csv",
        "robot_metric_summary.csv",
        "run_metric_summary.csv",
        "pairwise_metric_deltas.csv",
        "metric_definitions.csv",
        "report.md",
    ]:
        path = output_dir / name
        if path.exists():
            path.unlink()
    plots = output_dir / "plots"
    if plots.exists():
        shutil.rmtree(plots)


def main() -> int:
    args = parse_args()
    overrides = parse_world_overrides(args.world_override)
    runs = discover_runs(
        args.results_root,
        overrides,
        args.large_trajectory_world,
        args.large_trajectory_span_threshold,
        args.large_trajectory_length_threshold,
    )
    if not runs:
        print(f"No runs found under {args.results_root}", file=sys.stderr)
        return 2

    prepare_output_dir(args.output)
    inventory, canonical = build_inventory(runs, args.coverage_threshold)
    worlds = selected_worlds(canonical, args)
    if worlds:
        inventory_out = inventory.loc[inventory["world"].isin(worlds) | inventory["world"].isin(["auto", "unknown"])].copy()
        canonical = canonical.loc[canonical["world"].isin(worlds)].copy()
    else:
        inventory_out = inventory

    if not args.include_incomplete and not canonical.empty:
        canonical_for_tables = canonical.loc[canonical["valid_for_paper"]].copy()
    else:
        canonical_for_tables = canonical.copy()

    run_level = build_run_level_metrics(canonical_for_tables)
    robot_metric_cols = metric_columns(canonical_for_tables, PRIMARY_METRICS + GRAPH_METRICS)
    robot_summary = (
        summarize_group(canonical_for_tables, ["world", "algorithm", "robot_id"], robot_metric_cols)
        if not canonical_for_tables.empty
        else pd.DataFrame()
    )
    run_metric_cols = [
        col
        for col in run_level.columns
        if col not in {"world", "algorithm_dir", "algorithm", "run_id", "canonical_source", "valid_for_paper"}
        and pd.api.types.is_numeric_dtype(run_level[col])
    ]
    run_summary = (
        summarize_group(run_level, ["world", "algorithm"], run_metric_cols)
        if not run_level.empty
        else pd.DataFrame()
    )
    robot_pairwise = build_pairwise_deltas(robot_summary, ["world", "robot_id"])
    run_pairwise = build_pairwise_deltas(run_summary, ["world"])
    pairwise = pd.concat([robot_pairwise, run_pairwise], ignore_index=True) if not robot_pairwise.empty or not run_pairwise.empty else pd.DataFrame()

    inventory_out.to_csv(args.output / "run_inventory.csv", index=False)
    canonical.to_csv(args.output / "canonical_robot_metrics_all_selected.csv", index=False)
    canonical_for_tables.to_csv(args.output / "canonical_robot_metrics.csv", index=False)
    run_level.to_csv(args.output / "canonical_run_metrics.csv", index=False)
    robot_summary.to_csv(args.output / "robot_metric_summary.csv", index=False)
    run_summary.to_csv(args.output / "run_metric_summary.csv", index=False)
    pairwise.to_csv(args.output / "pairwise_metric_deltas.csv", index=False)
    metric_definitions().to_csv(args.output / "metric_definitions.csv", index=False)

    plot_paths: list[Path] = []
    if not args.no_plots:
        try:
            plot_paths.extend(plot_metric_bars(canonical_for_tables, args.output))
            for mrg_run, swarm_run, mrg_source, swarm_source in overlay_pairs(
                inventory,
                worlds,
                args,
            ):
                plot_paths.extend(
                    plot_trajectory_overlay(
                        mrg_run,
                        swarm_run,
                        mrg_source,
                        swarm_source,
                        args.output,
                    )
                )
        except Exception as exc:
            print(f"Plot generation failed: {exc}", file=sys.stderr)

    write_report(
        args.output,
        inventory_out,
        canonical_for_tables,
        run_level,
        run_summary,
        pairwise,
        plot_paths,
        args.coverage_threshold,
    )

    print(f"Wrote comparison outputs to {args.output}")
    if worlds:
        print(f"Worlds compared: {', '.join(worlds)}")
    else:
        print("No comparable worlds found.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
