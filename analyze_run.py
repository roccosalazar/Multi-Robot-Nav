#!/usr/bin/env python3
"""Analyze pose-graph snapshots against recorded ground truth.

The script supports both `results/swarm_multi` and `results/mrg_multi`.

New Swarm and MRG node CSVs include per-keyframe ROS timestamps. When those
are available, the script matches graph poses to ground truth directly in time
and ignores the manual association mode. Older Swarm CSVs without timestamps
fall back to associating graph keyframes by keyframe id over cumulative
ground-truth distance.
"""

from __future__ import annotations

import argparse
from functools import cached_property
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable

import numpy as np
import pandas as pd

try:
    from scipy.spatial.transform import Rotation, Slerp
except Exception as exc:  # pragma: no cover - fail with a friendly message.
    raise SystemExit(
        "scipy is required for SE(3) alignment and orientation metrics. "
        "Install it with: pip install scipy"
    ) from exc


POSITION_COLUMNS = ["position_x", "position_y", "position_z"]
QUAT_COLUMNS = [
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
]
MEASUREMENT_POSITION_COLUMNS = ["measurement_x", "measurement_y", "measurement_z"]
MEASUREMENT_QUAT_COLUMNS = [
    "measurement_qx",
    "measurement_qy",
    "measurement_qz",
    "measurement_qw",
]
DEFAULT_RPE_KEYFRAME_DELTAS = [1, 5, 10]
DEFAULT_RPE_DISTANCE_DELTAS_M = [1.0, 5.0, 10.0]


@dataclass
class GroundTruthTrajectory:
    robot_id: int
    robot_name: str
    positions: np.ndarray
    rotations: Rotation
    wall_time: np.ndarray
    ros_time: np.ndarray
    cumulative_distance: np.ndarray

    @cached_property
    def distance_samples(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return unique_axis_samples(
            self.cumulative_distance,
            self.positions,
            self.rotations.as_quat(),
        )

    @cached_property
    def distance_slerp(self) -> Slerp | None:
        axis, _, quats, _ = self.distance_samples
        if len(axis) == 1:
            return None
        return Slerp(axis, Rotation.from_quat(normalize_quaternions(quats)))

    @cached_property
    def index_axis(self) -> np.ndarray:
        return np.arange(len(self.positions), dtype=float)

    @cached_property
    def index_slerp(self) -> Slerp | None:
        if len(self.index_axis) == 1:
            return None
        return Slerp(self.index_axis, self.rotations)

    def sample_by_distance(
        self, distances: np.ndarray
    ) -> tuple[np.ndarray, Rotation, np.ndarray]:
        axis, pos, quats, indices = self.distance_samples
        clipped = np.clip(distances, axis[0], axis[-1])
        sampled_pos = interpolate_positions(axis, pos, clipped)
        if self.distance_slerp is None:
            sampled_rot = Rotation.from_quat(
                np.repeat(normalize_quaternions(quats)[:1], len(clipped), axis=0)
            )
        else:
            sampled_rot = self.distance_slerp(clipped)
        sampled_time = np.interp(clipped, axis, self.wall_time[indices])
        return sampled_pos, sampled_rot, sampled_time

    def sample_by_index(
        self, fractional_indices: np.ndarray
    ) -> tuple[np.ndarray, Rotation, np.ndarray]:
        clipped = np.clip(fractional_indices, self.index_axis[0], self.index_axis[-1])
        sampled_pos = interpolate_positions(self.index_axis, self.positions, clipped)
        if self.index_slerp is None:
            sampled_rot = Rotation.from_quat(
                np.repeat(self.rotations.as_quat()[:1], len(clipped), axis=0)
            )
        else:
            sampled_rot = self.index_slerp(clipped)
        sampled_time = np.interp(clipped, self.index_axis, self.wall_time)
        return sampled_pos, sampled_rot, sampled_time

    @cached_property
    def ros_time_samples(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return unique_axis_samples(
            self.ros_time,
            self.positions,
            self.rotations.as_quat(),
        )

    @cached_property
    def ros_time_slerp(self) -> Slerp | None:
        axis, _, quats, _ = self.ros_time_samples
        if len(axis) == 1:
            return None
        return Slerp(axis, Rotation.from_quat(normalize_quaternions(quats)))

    def sample_by_time(
        self, query_time: np.ndarray
    ) -> tuple[np.ndarray, Rotation, np.ndarray, np.ndarray]:
        axis, pos, quats, indices = self.ros_time_samples
        clipped = np.clip(query_time, axis[0], axis[-1])
        sampled_pos = interpolate_positions(axis, pos, clipped)
        if self.ros_time_slerp is None:
            sampled_rot = Rotation.from_quat(
                np.repeat(normalize_quaternions(quats)[:1], len(clipped), axis=0)
            )
        else:
            sampled_rot = self.ros_time_slerp(clipped)
        sampled_wall_time = np.interp(clipped, axis, self.wall_time[indices])
        sampled_progress = np.interp(
            clipped,
            axis,
            self.cumulative_distance[indices],
        ) / max(self.cumulative_distance[-1], 1e-12)
        return sampled_pos, sampled_rot, sampled_wall_time, sampled_progress


@dataclass
class Alignment:
    rotation: Rotation
    translation: np.ndarray

    def apply(
        self, positions: np.ndarray, rotations: Rotation
    ) -> tuple[np.ndarray, Rotation]:
        aligned_positions = self.rotation.apply(positions) + self.translation
        aligned_rotations = self.rotation * rotations
        return aligned_positions, aligned_rotations

    def matrix(self) -> np.ndarray:
        out = np.eye(4)
        out[:3, :3] = self.rotation.as_matrix()
        out[:3, 3] = self.translation
        return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze ATE/RPE and loop-closure effects for pose-graph snapshots "
            "recorded under results/swarm_multi or results/mrg_multi."
        )
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("results/swarm_multi"),
        help=(
            "Directory containing run folders, for example results/swarm_multi "
            "or results/mrg_multi."
        ),
    )
    parser.add_argument(
        "--run",
        type=Path,
        default=None,
        help=(
            "Specific run directory, for example "
            "results/swarm_multi/20260515_115918 or "
            "results/mrg_multi/20260515_133631."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory. Defaults to <run>/analysis_pose_graph.",
    )
    parser.add_argument(
        "--sources",
        nargs="*",
        default=None,
        help="Pose-graph source directories to analyze. Defaults to all with snapshots.csv.",
    )
    parser.add_argument(
        "--robots",
        nargs="*",
        default=None,
        help="Robot names/ids to analyze, for example r0 r2 or 0 2. Defaults to all ground truth files.",
    )
    parser.add_argument(
        "--alignment",
        choices=["first", "umeyama", "none"],
        default="umeyama",
        help=(
            "How to align graph poses to ground truth. Use 'umeyama' for "
            "post-optimization global ATE, 'first' for drift from the real "
            "initial pose, and 'none' for TF/frame debugging."
        ),
    )
    parser.add_argument(
        "--association",
        choices=["distance", "index", "graph-distance"],
        default="distance",
        help=(
            "How to associate keyframes to ground truth when node timestamps "
            "are unavailable. Ignored automatically when graph timestamps are "
            "present, as in MRG snapshots."
        ),
    )
    parser.add_argument(
        "--rpe-delta",
        type=int,
        default=None,
        help=(
            "Backward-compatible keyframe step for RPE. If provided, it is "
            "added to --rpe-keyframe-deltas."
        ),
    )
    parser.add_argument(
        "--rpe-keyframe-deltas",
        type=int,
        nargs="*",
        default=DEFAULT_RPE_KEYFRAME_DELTAS,
        help="Keyframe deltas used for RPE. Defaults to 1 5 10.",
    )
    parser.add_argument(
        "--rpe-distance-deltas",
        type=float,
        nargs="*",
        default=DEFAULT_RPE_DISTANCE_DELTAS_M,
        help="Metric deltas in meters used for distance-based RPE. Defaults to 1 5 10.",
    )
    parser.add_argument(
        "--loop-valid-trans-threshold-m",
        type=float,
        default=0.3,
        help="Translation threshold for a GT-valid inter-robot loop closure.",
    )
    parser.add_argument(
        "--loop-valid-rot-threshold-deg",
        type=float,
        default=3.0,
        help="Rotation threshold for a GT-valid inter-robot loop closure.",
    )
    parser.add_argument(
        "--loop-measurement-direction",
        choices=["auto", "as-stored", "inverted"],
        default="auto",
        help=(
            "Direction convention for inter-robot loop-closure measurements. "
            "'auto' infers a per-source convention from the final graph."
        ),
    )
    parser.add_argument(
        "--inter-loop-opportunity-radius-m",
        type=float,
        default=2.0,
        help=(
            "GT distance used to estimate real inter-robot loop-closure "
            "opportunities for recall."
        ),
    )
    parser.add_argument(
        "--event-window",
        type=int,
        default=5,
        help="Snapshots after a new loop closure used for event impact deltas.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip PNG plot generation.",
    )
    args = parser.parse_args()
    keyframe_deltas = list(args.rpe_keyframe_deltas or [])
    if args.rpe_delta is not None:
        keyframe_deltas.append(args.rpe_delta)
    args.rpe_keyframe_deltas = sorted(
        {int(delta) for delta in keyframe_deltas if int(delta) > 0}
    )
    if not args.rpe_keyframe_deltas:
        args.rpe_keyframe_deltas = [1]
    args.rpe_distance_deltas = sorted(
        {float(delta) for delta in (args.rpe_distance_deltas or []) if float(delta) > 0}
    )
    return args


def latest_run(results_root: Path) -> Path:
    if not results_root.exists():
        raise FileNotFoundError(f"Results root does not exist: {results_root}")
    candidates = [
        path
        for path in results_root.iterdir()
        if path.is_dir()
        and (path / "ground_truth").is_dir()
        and (path / "pose_graph").is_dir()
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No run directories with ground_truth/ and pose_graph/ in {results_root}"
        )
    return sorted(candidates, key=lambda path: (path.name, path.stat().st_mtime))[-1]


def robot_id_from_text(text: str) -> int | None:
    match = re.search(r"(?:^|[^0-9])r?(\d+)(?:[^0-9]|$)", text.lower())
    if match is None:
        return None
    return int(match.group(1))


def selected_robot_ids(raw: Iterable[str] | None) -> set[int] | None:
    if raw is None:
        return None
    out: set[int] = set()
    for item in raw:
        robot_id = robot_id_from_text(item)
        if robot_id is None:
            raise ValueError(f"Could not parse robot id from '{item}'")
        out.add(robot_id)
    return out


def normalize_quaternions(quats: np.ndarray) -> np.ndarray:
    quats = np.asarray(quats, dtype=float)
    norms = np.linalg.norm(quats, axis=1)
    bad = norms < 1e-12
    if np.any(bad):
        quats = quats.copy()
        quats[bad] = np.array([0.0, 0.0, 0.0, 1.0])
        norms = np.linalg.norm(quats, axis=1)
    return quats / norms[:, None]


def cumulative_distance(positions: np.ndarray) -> np.ndarray:
    if len(positions) == 0:
        return np.array([], dtype=float)
    if len(positions) == 1:
        return np.array([0.0], dtype=float)
    increments = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(increments)])


def unique_axis_samples(
    axis: np.ndarray,
    positions: np.ndarray,
    quats: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    axis = np.asarray(axis, dtype=float)
    if len(axis) == 0:
        raise ValueError("Cannot sample an empty trajectory")
    unique_axis, indices = np.unique(axis, return_index=True)
    if len(unique_axis) == 1:
        return unique_axis, positions[indices], quats[indices], indices
    return unique_axis, positions[indices], quats[indices], indices


def interpolate_positions(
    axis: np.ndarray, positions: np.ndarray, query: np.ndarray
) -> np.ndarray:
    if len(axis) == 1:
        return np.repeat(positions[:1], len(query), axis=0)
    return np.column_stack(
        [np.interp(query, axis, positions[:, dim]) for dim in range(3)]
    )


def interpolate_rotations(axis: np.ndarray, quats: np.ndarray, query: np.ndarray) -> Rotation:
    quats = normalize_quaternions(quats)
    if len(axis) == 1:
        return Rotation.from_quat(np.repeat(quats[:1], len(query), axis=0))
    slerp = Slerp(axis, Rotation.from_quat(quats))
    return slerp(query)


def load_ground_truth(run_dir: Path) -> dict[int, GroundTruthTrajectory]:
    gt_dir = run_dir / "ground_truth"
    trajectories: dict[int, GroundTruthTrajectory] = {}
    for csv_path in sorted(gt_dir.glob("ground_truth_*.csv")):
        robot_id = robot_id_from_text(csv_path.stem)
        df = pd.read_csv(csv_path)
        if df.empty:
            continue
        if robot_id is None:
            robot_id = robot_id_from_text(str(df["robot_name"].iloc[0]))
        if robot_id is None:
            continue
        positions = df[POSITION_COLUMNS].to_numpy(dtype=float)
        quats = normalize_quaternions(df[QUAT_COLUMNS].to_numpy(dtype=float))
        ros_time = (
            df["ros_time_sec"].to_numpy(dtype=float)
            + df["ros_time_nsec"].to_numpy(dtype=float) * 1e-9
        )
        trajectories[robot_id] = GroundTruthTrajectory(
            robot_id=robot_id,
            robot_name=str(df["robot_name"].iloc[0]),
            positions=positions,
            rotations=Rotation.from_quat(quats),
            wall_time=df["wall_time_unix_sec"].to_numpy(dtype=float),
            ros_time=ros_time,
            cumulative_distance=cumulative_distance(positions),
        )
    return trajectories


def discover_sources(run_dir: Path, source_filter: list[str] | None) -> list[Path]:
    pose_graph_dir = run_dir / "pose_graph"
    sources = [
        path
        for path in sorted(pose_graph_dir.iterdir())
        if path.is_dir() and (path / "snapshots.csv").is_file()
    ]
    if source_filter is None:
        return sources
    wanted = set(source_filter)
    return [path for path in sources if path.name in wanted]


def read_nodes(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.empty:
        return df

    if "robot_id" in df.columns and "keyframe_id" in df.columns:
        df["node_schema"] = "swarm"
        for column in ["robot_id", "keyframe_id"]:
            df[column] = pd.to_numeric(df[column], errors="coerce").astype("Int64")
    elif "keyframe_robot_name" in df.columns and "odom_counter" in df.columns:
        df["node_schema"] = "mrg"
        df["robot_id"] = (
            df["keyframe_robot_name"].astype(str).map(robot_id_from_text).astype("Int64")
        )
        df["keyframe_id"] = pd.to_numeric(df["odom_counter"], errors="coerce").astype("Int64")
    else:
        raise ValueError(f"Unrecognized node schema in {path}")

    if "stamp_sec" in df.columns and "stamp_nsec" in df.columns:
        stamp_sec = pd.to_numeric(df["stamp_sec"], errors="coerce")
        stamp_nsec = pd.to_numeric(df["stamp_nsec"], errors="coerce")
        df["graph_ros_time"] = stamp_sec + stamp_nsec * 1e-9

    for column in POSITION_COLUMNS + QUAT_COLUMNS:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    return df.dropna(subset=["robot_id", "keyframe_id", *POSITION_COLUMNS, *QUAT_COLUMNS])


def read_edges(path: Path, nodes: pd.DataFrame | None = None) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.empty:
        return df

    if {"from_robot_id", "to_robot_id"}.issubset(df.columns):
        for column in [
            "from_robot_id",
            "from_keyframe_id",
            "to_robot_id",
            "to_keyframe_id",
        ]:
            df[column] = pd.to_numeric(df[column], errors="coerce").astype("Int64")
        df["edge_schema"] = "swarm"
        return df

    if {"from_uuid", "to_uuid"}.issubset(df.columns) and nodes is not None and "uuid" in nodes.columns:
        uuid_to_robot = (
            nodes.dropna(subset=["uuid", "robot_id"])
            .drop_duplicates(subset=["uuid"])
            .set_index("uuid")["robot_id"]
            .to_dict()
        )
        uuid_to_keyframe = (
            nodes.dropna(subset=["uuid", "keyframe_id"])
            .drop_duplicates(subset=["uuid"])
            .set_index("uuid")["keyframe_id"]
            .to_dict()
        )
        df["from_robot_id"] = df["from_uuid"].map(uuid_to_robot).astype("Int64")
        df["to_robot_id"] = df["to_uuid"].map(uuid_to_robot).astype("Int64")
        df["from_keyframe_id"] = df["from_uuid"].map(uuid_to_keyframe).astype("Int64")
        df["to_keyframe_id"] = df["to_uuid"].map(uuid_to_keyframe).astype("Int64")
        relative_columns = [
            "relative_x",
            "relative_y",
            "relative_z",
            "relative_qx",
            "relative_qy",
            "relative_qz",
            "relative_qw",
        ]
        if set(relative_columns).issubset(df.columns):
            df["measurement_x"] = pd.to_numeric(df["relative_x"], errors="coerce")
            df["measurement_y"] = pd.to_numeric(df["relative_y"], errors="coerce")
            df["measurement_z"] = pd.to_numeric(df["relative_z"], errors="coerce")
            df["measurement_qx"] = pd.to_numeric(df["relative_qx"], errors="coerce")
            df["measurement_qy"] = pd.to_numeric(df["relative_qy"], errors="coerce")
            df["measurement_qz"] = pd.to_numeric(df["relative_qz"], errors="coerce")
            df["measurement_qw"] = pd.to_numeric(df["relative_qw"], errors="coerce")
        df["edge_schema"] = "mrg"
        return df

    raise ValueError(f"Unrecognized edge schema in {path}")


def edge_counts(edges: pd.DataFrame, robot_id: int | None = None) -> dict[str, int]:
    if edges.empty:
        return {
            "odom_edges": 0,
            "intra_loop_closures": 0,
            "inter_loop_closures": 0,
            "inter_robot_edges": 0,
            "loop_edges": 0,
            "other_edges": 0,
        }
    edge_type = edges["edge_type"].astype(str)
    from_robot = edges["from_robot_id"].fillna(-1).astype(int)
    to_robot = edges["to_robot_id"].fillna(-1).astype(int)
    if robot_id is None:
        edge_mask = pd.Series(True, index=edges.index)
    else:
        edge_mask = (from_robot == robot_id) | (to_robot == robot_id)
    same_robot = (from_robot == to_robot) & (from_robot >= 0)
    odom = edge_type == "odom"
    inter_robot = (edge_type == "inter_robot") | (
        (edge_type == "loop") & ~same_robot & (from_robot >= 0) & (to_robot >= 0)
    )
    intra_loop = (edge_type == "loop") & same_robot
    inter_loop = inter_robot
    loop_edges = intra_loop | inter_loop
    return {
        "odom_edges": int((odom & edge_mask).sum()),
        "intra_loop_closures": int((intra_loop & edge_mask).sum()),
        "inter_loop_closures": int((inter_loop & edge_mask).sum()),
        "inter_robot_edges": int((inter_robot & edge_mask).sum()),
        "loop_edges": int((loop_edges & edge_mask).sum()),
        "other_edges": int((~odom & ~loop_edges & edge_mask).sum()),
    }


def metric_name(prefix: str, name: str) -> str:
    return f"{prefix}_{name}" if prefix else name


def inter_robot_loop_mask(edges: pd.DataFrame) -> pd.Series:
    if edges.empty:
        return pd.Series(dtype=bool)
    edge_type = edges["edge_type"].astype(str)
    from_robot = edges["from_robot_id"].fillna(-1).astype(int)
    to_robot = edges["to_robot_id"].fillna(-1).astype(int)
    same_robot = (from_robot == to_robot) & (from_robot >= 0)
    return (edge_type == "inter_robot") | (
        (edge_type == "loop") & ~same_robot & (from_robot >= 0) & (to_robot >= 0)
    )


def incident_edge_mask(edges: pd.DataFrame, robot_id: int | None) -> pd.Series:
    if edges.empty:
        return pd.Series(dtype=bool)
    if robot_id is None:
        return pd.Series(True, index=edges.index)
    from_robot = edges["from_robot_id"].fillna(-1).astype(int)
    to_robot = edges["to_robot_id"].fillna(-1).astype(int)
    return (from_robot == robot_id) | (to_robot == robot_id)


def node_pose_lookup(nodes: pd.DataFrame) -> dict[tuple[int, int], tuple[np.ndarray, Rotation]]:
    lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]] = {}
    if nodes.empty:
        return lookup
    robot_ids = nodes["robot_id"].astype(int).to_numpy()
    keyframe_ids = nodes["keyframe_id"].astype(int).to_numpy()
    positions = nodes[POSITION_COLUMNS].to_numpy(dtype=float)
    quats = normalize_quaternions(nodes[QUAT_COLUMNS].to_numpy(dtype=float))
    rotations = Rotation.from_quat(quats)
    for idx in range(len(nodes)):
        lookup[(int(robot_ids[idx]), int(keyframe_ids[idx]))] = (
            positions[idx],
            rotations[idx],
        )
    return lookup


def measurement_pose(row: pd.Series) -> tuple[np.ndarray, Rotation] | None:
    if not set(MEASUREMENT_POSITION_COLUMNS + MEASUREMENT_QUAT_COLUMNS).issubset(
        row.index
    ):
        return None
    position = row[MEASUREMENT_POSITION_COLUMNS].to_numpy(dtype=float)
    quat = normalize_quaternions(
        row[MEASUREMENT_QUAT_COLUMNS].to_numpy(dtype=float).reshape(1, 4)
    )[0]
    if not np.isfinite(position).all() or not np.isfinite(quat).all():
        return None
    return position, Rotation.from_quat(quat)


def relative_pose_from_lookup(
    lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    from_key: tuple[int, int],
    to_key: tuple[int, int],
) -> tuple[np.ndarray, Rotation] | None:
    if from_key not in lookup or to_key not in lookup:
        return None
    from_position, from_rotation = lookup[from_key]
    to_position, to_rotation = lookup[to_key]
    rel_position = from_rotation.inv().apply(to_position - from_position)
    rel_rotation = from_rotation.inv() * to_rotation
    return rel_position, rel_rotation


def pose_residual(
    estimated: tuple[np.ndarray, Rotation],
    reference: tuple[np.ndarray, Rotation],
) -> tuple[float, float]:
    estimated_position, estimated_rotation = estimated
    reference_position, reference_rotation = reference
    trans_error = float(np.linalg.norm(estimated_position - reference_position))
    rot_error = float(
        np.degrees((reference_rotation.inv() * estimated_rotation).magnitude())
    )
    return trans_error, rot_error


def invert_relative_pose(
    pose: tuple[np.ndarray, Rotation],
) -> tuple[np.ndarray, Rotation]:
    position, rotation = pose
    inverse_rotation = rotation.inv()
    return -inverse_rotation.apply(position), inverse_rotation


def wrap_degrees(angle_deg: float) -> float:
    return float((angle_deg + 180.0) % 360.0 - 180.0)


def yaw_deg(rotation: Rotation) -> float:
    return float(rotation.as_euler("xyz", degrees=True)[2])


def planar_pose_residual(
    estimated: tuple[np.ndarray, Rotation],
    reference: tuple[np.ndarray, Rotation],
) -> tuple[float, float]:
    estimated_position, estimated_rotation = estimated
    reference_position, reference_rotation = reference
    trans_error = float(np.linalg.norm(estimated_position[:2] - reference_position[:2]))
    yaw_error = abs(wrap_degrees(yaw_deg(estimated_rotation) - yaw_deg(reference_rotation)))
    return trans_error, yaw_error


def diagnostic_score(
    trans_error: float,
    rot_error: float,
    trans_threshold_m: float,
    rot_threshold_deg: float,
) -> float:
    trans_scale = max(trans_threshold_m, 1e-12)
    rot_scale = max(rot_threshold_deg, 1e-12)
    return float(math.hypot(trans_error / trans_scale, rot_error / rot_scale))


def orient_relative_pose(
    pose: tuple[np.ndarray, Rotation],
    measurement_direction: str,
) -> tuple[np.ndarray, Rotation]:
    if measurement_direction == "inverted":
        return invert_relative_pose(pose)
    return pose


def map_consistency_metrics(
    edges: pd.DataFrame,
    nodes: pd.DataFrame,
    prefix: str,
    robot_id: int | None = None,
) -> dict[str, float | int]:
    if edges.empty or nodes.empty:
        return trans_rot_empty_stats(prefix)
    pose_lookup = node_pose_lookup(nodes)
    mask = inter_robot_loop_mask(edges) & incident_edge_mask(edges, robot_id)
    trans_errors: list[float] = []
    rot_errors: list[float] = []
    for _, edge in edges[mask].iterrows():
        from_key = (int(edge["from_robot_id"]), int(edge["from_keyframe_id"]))
        to_key = (int(edge["to_robot_id"]), int(edge["to_keyframe_id"]))
        graph_relative = relative_pose_from_lookup(pose_lookup, from_key, to_key)
        measurement_relative = measurement_pose(edge)
        if graph_relative is None or measurement_relative is None:
            continue
        trans_error, rot_error = pose_residual(graph_relative, measurement_relative)
        trans_errors.append(trans_error)
        rot_errors.append(rot_error)
    return trans_rot_stats(np.asarray(trans_errors), np.asarray(rot_errors), prefix)


def map_consistency_metrics_all(
    edges: pd.DataFrame,
    nodes: pd.DataFrame,
    measurement_direction: str,
) -> tuple[dict[str, float | int], dict[int, dict[str, float | int]]]:
    graph_metrics = trans_rot_empty_stats("graph_map_consistency")
    graph_metrics["global_map_consistency_error_m"] = math.nan
    incident_metrics: dict[int, dict[str, float | int]] = {
        int(robot_id): {
            **trans_rot_empty_stats("map_consistency"),
            "map_consistency_error_m": math.nan,
        }
        for robot_id in sorted(set(nodes["robot_id"].astype(int))) if not nodes.empty
    }
    if edges.empty or nodes.empty:
        return graph_metrics, incident_metrics

    pose_lookup = node_pose_lookup(nodes)
    mask = inter_robot_loop_mask(edges)
    graph_trans: list[float] = []
    graph_rot: list[float] = []
    incident_trans: dict[int, list[float]] = {}
    incident_rot: dict[int, list[float]] = {}
    for _, edge in edges[mask].iterrows():
        from_robot = int(edge["from_robot_id"])
        to_robot = int(edge["to_robot_id"])
        from_key = (from_robot, int(edge["from_keyframe_id"]))
        to_key = (to_robot, int(edge["to_keyframe_id"]))
        graph_relative = relative_pose_from_lookup(pose_lookup, from_key, to_key)
        measurement_relative = measurement_pose(edge)
        if graph_relative is None or measurement_relative is None:
            continue
        measurement_relative = orient_relative_pose(
            measurement_relative,
            measurement_direction,
        )
        trans_error, rot_error = pose_residual(graph_relative, measurement_relative)
        graph_trans.append(trans_error)
        graph_rot.append(rot_error)
        for current_robot in {from_robot, to_robot}:
            incident_trans.setdefault(current_robot, []).append(trans_error)
            incident_rot.setdefault(current_robot, []).append(rot_error)

    graph_metrics = trans_rot_stats(
        np.asarray(graph_trans),
        np.asarray(graph_rot),
        "graph_map_consistency",
    )
    graph_metrics["global_map_consistency_error_m"] = graph_metrics.get(
        "graph_map_consistency_trans_rmse_m",
        math.nan,
    )
    for robot_id in incident_metrics:
        metrics = trans_rot_stats(
            np.asarray(incident_trans.get(robot_id, [])),
            np.asarray(incident_rot.get(robot_id, [])),
            "map_consistency",
        )
        metrics["map_consistency_error_m"] = metrics.get(
            "map_consistency_trans_rmse_m",
            math.nan,
        )
        incident_metrics[robot_id] = metrics
    return graph_metrics, incident_metrics


def graph_correction_metrics(
    previous_nodes: pd.DataFrame | None,
    current_nodes: pd.DataFrame,
    robot_id: int,
    prefix: str = "graph_correction",
) -> dict[str, float | int]:
    if previous_nodes is None or previous_nodes.empty or current_nodes.empty:
        return trans_rot_empty_stats(prefix)
    previous_lookup = node_pose_lookup(
        previous_nodes[previous_nodes["robot_id"].astype(int) == robot_id]
    )
    current_lookup = node_pose_lookup(
        current_nodes[current_nodes["robot_id"].astype(int) == robot_id]
    )
    common_keys = sorted(set(previous_lookup).intersection(current_lookup))
    if not common_keys:
        return trans_rot_empty_stats(prefix)

    trans_errors: list[float] = []
    rot_errors: list[float] = []
    for key in common_keys:
        previous_position, previous_rotation = previous_lookup[key]
        current_position, current_rotation = current_lookup[key]
        trans_errors.append(float(np.linalg.norm(current_position - previous_position)))
        rot_errors.append(
            float(np.degrees((previous_rotation.inv() * current_rotation).magnitude()))
        )
    return trans_rot_stats(np.asarray(trans_errors), np.asarray(rot_errors), prefix)


def compute_max_keyframes(
    run_dir: Path,
    sources: list[Path],
) -> dict[tuple[str, int], int]:
    max_keyframes: dict[tuple[str, int], int] = {}
    for source_dir in sources:
        snapshots = pd.read_csv(source_dir / "snapshots.csv")
        for _, snapshot in snapshots.iterrows():
            nodes_path = run_dir / str(snapshot["nodes_file"])
            if not nodes_path.exists():
                continue
            nodes = read_nodes(nodes_path)
            if nodes.empty:
                continue
            for robot_id, robot_nodes in nodes.groupby("robot_id"):
                key = (source_dir.name, int(robot_id))
                current = max_keyframes.get(key, -1)
                max_keyframes[key] = max(
                    current,
                    int(robot_nodes["keyframe_id"].max()),
                )
    return max_keyframes


def prepare_graph_trajectory(
    nodes: pd.DataFrame,
    robot_id: int,
) -> tuple[np.ndarray, np.ndarray, Rotation, np.ndarray | None]:
    robot_nodes = nodes[nodes["robot_id"].astype(int) == robot_id].copy()
    robot_nodes = robot_nodes.sort_values("keyframe_id")
    keyframe_ids = robot_nodes["keyframe_id"].astype(int).to_numpy()
    positions = robot_nodes[POSITION_COLUMNS].to_numpy(dtype=float)
    quats = normalize_quaternions(robot_nodes[QUAT_COLUMNS].to_numpy(dtype=float))
    graph_ros_time = None
    if "graph_ros_time" in robot_nodes.columns:
        candidate_time = robot_nodes["graph_ros_time"].to_numpy(dtype=float)
        if np.isfinite(candidate_time).any():
            graph_ros_time = candidate_time
    return keyframe_ids, positions, Rotation.from_quat(quats), graph_ros_time


def matched_ground_truth_by_fallback(
    gt: GroundTruthTrajectory,
    keyframe_ids: np.ndarray,
    graph_positions: np.ndarray,
    max_keyframe_id: int,
    association: str,
) -> tuple[np.ndarray, Rotation, np.ndarray, np.ndarray]:
    if association == "index":
        if max_keyframe_id <= 0:
            fractional_indices = np.zeros_like(keyframe_ids, dtype=float)
        else:
            fractional_indices = (
                keyframe_ids.astype(float)
                / float(max_keyframe_id)
                * float(len(gt.positions) - 1)
            )
        positions, rotations, times = gt.sample_by_index(fractional_indices)
        progress = fractional_indices / max(1.0, float(len(gt.positions) - 1))
        return positions, rotations, times, progress

    if association == "graph-distance":
        graph_dist = cumulative_distance(graph_positions)
        positions, rotations, times = gt.sample_by_distance(graph_dist)
        progress = graph_dist / max(gt.cumulative_distance[-1], 1e-12)
        return positions, rotations, times, progress

    if max_keyframe_id <= 0:
        gt_dist = np.zeros_like(keyframe_ids, dtype=float)
    else:
        gt_dist = (
            keyframe_ids.astype(float)
            / float(max_keyframe_id)
            * float(gt.cumulative_distance[-1])
        )
    positions, rotations, times = gt.sample_by_distance(gt_dist)
    progress = gt_dist / max(gt.cumulative_distance[-1], 1e-12)
    return positions, rotations, times, progress


def matched_ground_truth(
    gt: GroundTruthTrajectory,
    keyframe_ids: np.ndarray,
    graph_positions: np.ndarray,
    graph_ros_time: np.ndarray | None,
    max_keyframe_id: int,
    association: str,
) -> tuple[np.ndarray, Rotation, np.ndarray, np.ndarray]:
    if graph_ros_time is None or len(graph_ros_time) != len(keyframe_ids):
        return matched_ground_truth_by_fallback(
            gt,
            keyframe_ids,
            graph_positions,
            max_keyframe_id,
            association,
        )

    time_mask = np.isfinite(graph_ros_time)
    if time_mask.all():
        return gt.sample_by_time(graph_ros_time)

    positions, rotations, times, progress = matched_ground_truth_by_fallback(
        gt,
        keyframe_ids,
        graph_positions,
        max_keyframe_id,
        association,
    )
    if not time_mask.any():
        return positions, rotations, times, progress

    timed_positions, timed_rotations, timed_times, timed_progress = gt.sample_by_time(
        graph_ros_time[time_mask]
    )
    positions[time_mask] = timed_positions
    quats = rotations.as_quat()
    quats[time_mask] = timed_rotations.as_quat()
    times[time_mask] = timed_times
    progress[time_mask] = timed_progress
    return positions, Rotation.from_quat(normalize_quaternions(quats)), times, progress


def association_used(
    graph_ros_time: np.ndarray | None,
    keyframe_ids: np.ndarray,
    fallback_association: str,
) -> str:
    if graph_ros_time is None or len(graph_ros_time) != len(keyframe_ids):
        return fallback_association
    time_count = int(np.isfinite(graph_ros_time).sum())
    if time_count == len(keyframe_ids):
        return "ros_time"
    if time_count > 0:
        return f"ros_time_partial_{fallback_association}"
    return fallback_association


def build_ground_truth_pose_lookup(
    nodes: pd.DataFrame,
    ground_truth: dict[int, GroundTruthTrajectory],
    max_keyframes: dict[tuple[str, int], int],
    source: str,
    association: str,
) -> dict[tuple[int, int], tuple[np.ndarray, Rotation]]:
    lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]] = {}
    if nodes.empty:
        return lookup
    for robot_id in sorted(set(nodes["robot_id"].astype(int))):
        if robot_id not in ground_truth:
            continue
        keyframe_ids, graph_positions, _, graph_ros_time = prepare_graph_trajectory(
            nodes,
            robot_id,
        )
        if len(keyframe_ids) == 0:
            continue
        gt_positions, gt_rotations, _, _ = matched_ground_truth(
            ground_truth[robot_id],
            keyframe_ids,
            graph_positions,
            graph_ros_time,
            max_keyframes.get((source, robot_id), 0),
            association,
        )
        for idx, keyframe_id in enumerate(keyframe_ids):
            lookup[(robot_id, int(keyframe_id))] = (
                gt_positions[idx],
                gt_rotations[idx],
            )
    return lookup


def inter_robot_opportunity_count(
    nodes: pd.DataFrame,
    gt_lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    radius_m: float,
    robot_id: int | None = None,
) -> int:
    total, incident = inter_robot_opportunity_counts(nodes, gt_lookup, radius_m)
    if robot_id is None:
        return total
    return incident.get(robot_id, 0)


def inter_robot_opportunity_counts(
    nodes: pd.DataFrame,
    gt_lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    radius_m: float,
) -> tuple[int, dict[int, int]]:
    if radius_m <= 0:
        return 0, {}
    positions_by_robot: dict[int, list[np.ndarray]] = {}
    for _, row in nodes.iterrows():
        current_robot = int(row["robot_id"])
        key = (current_robot, int(row["keyframe_id"]))
        if key not in gt_lookup:
            continue
        positions_by_robot.setdefault(current_robot, []).append(gt_lookup[key][0])

    total = 0
    incident: dict[int, int] = {}
    robot_ids = sorted(positions_by_robot)
    for idx, left_robot in enumerate(robot_ids):
        for right_robot in robot_ids[idx + 1 :]:
            left_positions = np.asarray(positions_by_robot[left_robot], dtype=float)
            right_positions = np.asarray(positions_by_robot[right_robot], dtype=float)
            if len(left_positions) == 0 or len(right_positions) == 0:
                continue
            deltas = left_positions[:, None, :] - right_positions[None, :, :]
            distances = np.linalg.norm(deltas, axis=2)
            pair_count = int((distances <= radius_m).sum())
            total += pair_count
            incident[left_robot] = incident.get(left_robot, 0) + pair_count
            incident[right_robot] = incident.get(right_robot, 0) + pair_count
    return total, incident


def inter_robot_loop_quality_metrics(
    edges: pd.DataFrame,
    nodes: pd.DataFrame,
    gt_lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    *,
    trans_threshold_m: float,
    rot_threshold_deg: float,
    opportunity_radius_m: float,
    prefix: str,
    robot_id: int | None = None,
    opportunity_count: int | None = None,
) -> dict[str, float | int]:
    out: dict[str, float | int] = {}
    names = [
        "evaluated_inter_robot_loop_closures",
        "valid_inter_robot_loop_closures",
        "invalid_inter_robot_loop_closures",
        "inter_robot_loop_closure_precision",
        "real_inter_robot_loop_opportunities",
        "found_real_inter_robot_loop_opportunities",
        "inter_robot_loop_closure_recall",
        "inter_robot_loop_gt_distance_mean_m",
        "inter_robot_loop_gt_distance_p95_m",
        "inter_robot_loop_measurement_trans_error_rmse_m",
        "inter_robot_loop_measurement_rot_error_rmse_deg",
    ]
    for name in names:
        out[metric_name(prefix, name)] = 0 if "closures" in name or "opportunities" in name else math.nan

    opportunities = (
        opportunity_count
        if opportunity_count is not None
        else inter_robot_opportunity_count(
            nodes,
            gt_lookup,
            opportunity_radius_m,
            robot_id=robot_id,
        )
    )
    out[metric_name(prefix, "real_inter_robot_loop_opportunities")] = opportunities

    if edges.empty:
        return out

    mask = inter_robot_loop_mask(edges) & incident_edge_mask(edges, robot_id)
    evaluated = 0
    valid = 0
    found_opportunities = 0
    trans_errors: list[float] = []
    rot_errors: list[float] = []
    gt_distances: list[float] = []
    for _, edge in edges[mask].iterrows():
        from_key = (int(edge["from_robot_id"]), int(edge["from_keyframe_id"]))
        to_key = (int(edge["to_robot_id"]), int(edge["to_keyframe_id"]))
        gt_relative = relative_pose_from_lookup(gt_lookup, from_key, to_key)
        measurement_relative = measurement_pose(edge)
        if gt_relative is None or measurement_relative is None:
            continue

        evaluated += 1
        trans_error, rot_error = pose_residual(measurement_relative, gt_relative)
        trans_errors.append(trans_error)
        rot_errors.append(rot_error)
        from_position = gt_lookup[from_key][0]
        to_position = gt_lookup[to_key][0]
        gt_distance = float(np.linalg.norm(to_position - from_position))
        gt_distances.append(gt_distance)
        is_valid = trans_error <= trans_threshold_m and rot_error <= rot_threshold_deg
        if is_valid:
            valid += 1
            if gt_distance <= opportunity_radius_m:
                found_opportunities += 1

    invalid = evaluated - valid
    precision = valid / evaluated if evaluated > 0 else math.nan
    recall = found_opportunities / opportunities if opportunities > 0 else math.nan
    out.update(
        {
            metric_name(prefix, "evaluated_inter_robot_loop_closures"): evaluated,
            metric_name(prefix, "valid_inter_robot_loop_closures"): valid,
            metric_name(prefix, "invalid_inter_robot_loop_closures"): invalid,
            metric_name(prefix, "inter_robot_loop_closure_precision"): precision,
            metric_name(prefix, "found_real_inter_robot_loop_opportunities"): found_opportunities,
            metric_name(prefix, "inter_robot_loop_closure_recall"): recall,
        }
    )
    if trans_errors:
        trans = np.asarray(trans_errors)
        rot = np.asarray(rot_errors)
        distances = np.asarray(gt_distances)
        out.update(
            {
                metric_name(prefix, "inter_robot_loop_gt_distance_mean_m"): float(
                    np.mean(distances)
                ),
                metric_name(prefix, "inter_robot_loop_gt_distance_p95_m"): float(
                    np.percentile(distances, 95)
                ),
                metric_name(prefix, "inter_robot_loop_measurement_trans_error_rmse_m"): float(
                    np.sqrt(np.mean(trans**2))
                ),
                metric_name(prefix, "inter_robot_loop_measurement_rot_error_rmse_deg"): float(
                    np.sqrt(np.mean(rot**2))
                ),
            }
        )
    return out


def empty_loop_quality_metrics(prefix: str, opportunities: int) -> dict[str, float | int]:
    return {
        metric_name(prefix, "evaluated_inter_robot_loop_closures"): 0,
        metric_name(prefix, "valid_inter_robot_loop_closures"): 0,
        metric_name(prefix, "invalid_inter_robot_loop_closures"): 0,
        metric_name(prefix, "inter_robot_loop_closure_precision"): math.nan,
        metric_name(prefix, "real_inter_robot_loop_opportunities"): opportunities,
        metric_name(prefix, "found_real_inter_robot_loop_opportunities"): 0,
        metric_name(prefix, "inter_robot_loop_closure_recall"): (
            0.0 if opportunities > 0 else math.nan
        ),
        metric_name(prefix, "inter_robot_loop_gt_distance_mean_m"): math.nan,
        metric_name(prefix, "inter_robot_loop_gt_distance_p95_m"): math.nan,
        metric_name(prefix, "inter_robot_loop_measurement_trans_error_rmse_m"): math.nan,
        metric_name(prefix, "inter_robot_loop_measurement_rot_error_rmse_deg"): math.nan,
    }


def loop_quality_from_records(
    records: list[dict[str, float | bool]],
    opportunities: int,
    prefix: str,
) -> dict[str, float | int]:
    if not records:
        return empty_loop_quality_metrics(prefix, opportunities)

    evaluated = len(records)
    valid = int(sum(1 for record in records if bool(record["valid"])))
    found_opportunities = int(
        sum(1 for record in records if bool(record["valid"]) and bool(record["opportunity"]))
    )
    trans = np.asarray([float(record["trans_error"]) for record in records])
    rot = np.asarray([float(record["rot_error"]) for record in records])
    distances = np.asarray([float(record["gt_distance"]) for record in records])
    return {
        metric_name(prefix, "evaluated_inter_robot_loop_closures"): evaluated,
        metric_name(prefix, "valid_inter_robot_loop_closures"): valid,
        metric_name(prefix, "invalid_inter_robot_loop_closures"): evaluated - valid,
        metric_name(prefix, "inter_robot_loop_closure_precision"): valid / evaluated,
        metric_name(prefix, "real_inter_robot_loop_opportunities"): opportunities,
        metric_name(prefix, "found_real_inter_robot_loop_opportunities"): found_opportunities,
        metric_name(prefix, "inter_robot_loop_closure_recall"): (
            found_opportunities / opportunities if opportunities > 0 else math.nan
        ),
        metric_name(prefix, "inter_robot_loop_gt_distance_mean_m"): float(
            np.mean(distances)
        ),
        metric_name(prefix, "inter_robot_loop_gt_distance_p95_m"): float(
            np.percentile(distances, 95)
        ),
        metric_name(prefix, "inter_robot_loop_measurement_trans_error_rmse_m"): float(
            np.sqrt(np.mean(trans**2))
        ),
        metric_name(prefix, "inter_robot_loop_measurement_rot_error_rmse_deg"): float(
            np.sqrt(np.mean(rot**2))
        ),
    }


def inter_robot_loop_quality_metrics_all(
    edges: pd.DataFrame,
    nodes: pd.DataFrame,
    gt_lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    *,
    trans_threshold_m: float,
    rot_threshold_deg: float,
    measurement_direction: str,
    opportunity_radius_m: float,
    graph_opportunities: int,
    incident_opportunities: dict[int, int],
) -> tuple[dict[str, float | int], dict[int, dict[str, float | int]]]:
    robot_ids = sorted(set(nodes["robot_id"].astype(int))) if not nodes.empty else []
    graph_records: list[dict[str, float | bool]] = []
    incident_records: dict[int, list[dict[str, float | bool]]] = {
        int(robot_id): [] for robot_id in robot_ids
    }
    if not edges.empty:
        mask = inter_robot_loop_mask(edges)
        for _, edge in edges[mask].iterrows():
            from_robot = int(edge["from_robot_id"])
            to_robot = int(edge["to_robot_id"])
            from_key = (from_robot, int(edge["from_keyframe_id"]))
            to_key = (to_robot, int(edge["to_keyframe_id"]))
            gt_relative = relative_pose_from_lookup(gt_lookup, from_key, to_key)
            measurement_relative = measurement_pose(edge)
            if gt_relative is None or measurement_relative is None:
                continue
            measurement_relative = orient_relative_pose(
                measurement_relative,
                measurement_direction,
            )

            trans_error, rot_error = pose_residual(measurement_relative, gt_relative)
            from_position = gt_lookup[from_key][0]
            to_position = gt_lookup[to_key][0]
            gt_distance = float(np.linalg.norm(to_position - from_position))
            record = {
                "trans_error": trans_error,
                "rot_error": rot_error,
                "gt_distance": gt_distance,
                "valid": trans_error <= trans_threshold_m
                and rot_error <= rot_threshold_deg,
                "opportunity": gt_distance <= opportunity_radius_m,
            }
            graph_records.append(record)
            for current_robot in {from_robot, to_robot}:
                incident_records.setdefault(current_robot, []).append(record)

    graph_metrics = loop_quality_from_records(
        graph_records,
        graph_opportunities,
        "graph",
    )
    incident_metrics = {
        int(robot_id): loop_quality_from_records(
            incident_records.get(int(robot_id), []),
            incident_opportunities.get(int(robot_id), 0),
            "",
        )
        for robot_id in robot_ids
    }
    return graph_metrics, incident_metrics


def infer_loop_measurement_direction(
    edges: pd.DataFrame,
    gt_lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    *,
    trans_threshold_m: float,
    rot_threshold_deg: float,
) -> str:
    if edges.empty:
        return "as-stored"

    current_scores: list[float] = []
    inverted_scores: list[float] = []
    for _, edge in edges[inter_robot_loop_mask(edges)].iterrows():
        from_key = (int(edge["from_robot_id"]), int(edge["from_keyframe_id"]))
        to_key = (int(edge["to_robot_id"]), int(edge["to_keyframe_id"]))
        gt_relative = relative_pose_from_lookup(gt_lookup, from_key, to_key)
        measurement_relative = measurement_pose(edge)
        if gt_relative is None or measurement_relative is None:
            continue
        current_trans, current_rot = pose_residual(measurement_relative, gt_relative)
        inverted_trans, inverted_rot = pose_residual(
            invert_relative_pose(measurement_relative),
            gt_relative,
        )
        current_scores.append(
            diagnostic_score(
                current_trans,
                current_rot,
                trans_threshold_m,
                rot_threshold_deg,
            )
        )
        inverted_scores.append(
            diagnostic_score(
                inverted_trans,
                inverted_rot,
                trans_threshold_m,
                rot_threshold_deg,
            )
        )
    if not current_scores:
        return "as-stored"
    if float(np.median(inverted_scores)) < float(np.median(current_scores)):
        return "inverted"
    return "as-stored"


def inter_robot_loop_diagnostics(
    *,
    source: str,
    snapshot: pd.Series,
    edges: pd.DataFrame,
    gt_lookup: dict[tuple[int, int], tuple[np.ndarray, Rotation]],
    selected_measurement_direction: str,
    trans_threshold_m: float,
    rot_threshold_deg: float,
    opportunity_radius_m: float,
) -> list[dict[str, object]]:
    if edges.empty:
        return []

    rows: list[dict[str, object]] = []
    mask = inter_robot_loop_mask(edges)
    for _, edge in edges[mask].iterrows():
        from_robot = int(edge["from_robot_id"])
        to_robot = int(edge["to_robot_id"])
        from_key = (from_robot, int(edge["from_keyframe_id"]))
        to_key = (to_robot, int(edge["to_keyframe_id"]))
        gt_relative = relative_pose_from_lookup(gt_lookup, from_key, to_key)
        measurement_relative = measurement_pose(edge)
        if gt_relative is None or measurement_relative is None:
            continue

        inverted_measurement = invert_relative_pose(measurement_relative)
        current_trans, current_rot = pose_residual(measurement_relative, gt_relative)
        inverted_trans, inverted_rot = pose_residual(inverted_measurement, gt_relative)
        planar_trans, planar_yaw = planar_pose_residual(measurement_relative, gt_relative)
        planar_inverted_trans, planar_inverted_yaw = planar_pose_residual(
            inverted_measurement,
            gt_relative,
        )
        current_valid = current_trans <= trans_threshold_m and current_rot <= rot_threshold_deg
        inverted_valid = inverted_trans <= trans_threshold_m and inverted_rot <= rot_threshold_deg
        planar_valid = planar_trans <= trans_threshold_m and planar_yaw <= rot_threshold_deg
        planar_inverted_valid = (
            planar_inverted_trans <= trans_threshold_m
            and planar_inverted_yaw <= rot_threshold_deg
        )
        scores = {
            "current_se3": diagnostic_score(
                current_trans,
                current_rot,
                trans_threshold_m,
                rot_threshold_deg,
            ),
            "inverted_se3": diagnostic_score(
                inverted_trans,
                inverted_rot,
                trans_threshold_m,
                rot_threshold_deg,
            ),
            "current_planar": diagnostic_score(
                planar_trans,
                planar_yaw,
                trans_threshold_m,
                rot_threshold_deg,
            ),
            "inverted_planar": diagnostic_score(
                planar_inverted_trans,
                planar_inverted_yaw,
                trans_threshold_m,
                rot_threshold_deg,
            ),
        }
        best_mode = min(scores, key=scores.get)
        from_position = gt_lookup[from_key][0]
        to_position = gt_lookup[to_key][0]
        gt_distance = float(np.linalg.norm(to_position - from_position))

        rows.append(
            {
                "source": source,
                "snapshot_seq": int(snapshot["snapshot_seq"]),
                "wall_time_unix_sec": float(snapshot["wall_time_unix_sec"]),
                "edge_type": str(edge["edge_type"]),
                "from_robot_id": from_robot,
                "from_keyframe_id": int(edge["from_keyframe_id"]),
                "to_robot_id": to_robot,
                "to_keyframe_id": int(edge["to_keyframe_id"]),
                "gt_distance_m": gt_distance,
                "is_gt_opportunity": gt_distance <= opportunity_radius_m,
                "threshold_trans_m": trans_threshold_m,
                "threshold_rot_deg": rot_threshold_deg,
                "selected_measurement_direction": selected_measurement_direction,
                "current_trans_error_m": current_trans,
                "current_rot_error_deg": current_rot,
                "current_valid": current_valid,
                "inverted_trans_error_m": inverted_trans,
                "inverted_rot_error_deg": inverted_rot,
                "inverted_valid": inverted_valid,
                "planar_trans_error_m": planar_trans,
                "planar_yaw_error_deg": planar_yaw,
                "planar_valid": planar_valid,
                "planar_inverted_trans_error_m": planar_inverted_trans,
                "planar_inverted_yaw_error_deg": planar_inverted_yaw,
                "planar_inverted_valid": planar_inverted_valid,
                "best_diagnostic_mode": best_mode,
                "best_diagnostic_score": scores[best_mode],
                "current_score": scores["current_se3"],
                "inverted_score": scores["inverted_se3"],
                "planar_score": scores["current_planar"],
                "planar_inverted_score": scores["inverted_planar"],
            }
        )
    return rows


def first_finite(values: np.ndarray | None) -> float:
    if values is None:
        return math.nan
    finite_values = values[np.isfinite(values)]
    if len(finite_values) == 0:
        return math.nan
    return float(finite_values[0])


def last_finite(values: np.ndarray | None) -> float:
    if values is None:
        return math.nan
    finite_values = values[np.isfinite(values)]
    if len(finite_values) == 0:
        return math.nan
    return float(finite_values[-1])


def align_trajectories(
    graph_positions: np.ndarray,
    graph_rotations: Rotation,
    gt_positions: np.ndarray,
    gt_rotations: Rotation,
    mode: str,
) -> Alignment:
    if mode == "none":
        return Alignment(Rotation.identity(), np.zeros(3))

    if mode == "first":
        r_align = gt_rotations[0] * graph_rotations[0].inv()
        t_align = gt_positions[0] - r_align.apply(graph_positions[0])
        return Alignment(r_align, t_align)

    if len(graph_positions) < 3:
        return align_trajectories(
            graph_positions,
            graph_rotations,
            gt_positions,
            gt_rotations,
            "first",
        )

    source_center = graph_positions.mean(axis=0)
    target_center = gt_positions.mean(axis=0)
    source_zero = graph_positions - source_center
    target_zero = gt_positions - target_center
    covariance = source_zero.T @ target_zero / len(graph_positions)
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation_matrix = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation_matrix) < 0:
        vt_matrix[-1, :] *= -1.0
        rotation_matrix = vt_matrix.T @ u_matrix.T
    r_align = Rotation.from_matrix(rotation_matrix)
    t_align = target_center - r_align.apply(source_center)
    return Alignment(r_align, t_align)


def rotation_errors_deg(
    aligned_rotations: Rotation,
    gt_rotations: Rotation,
) -> np.ndarray:
    return np.degrees((gt_rotations.inv() * aligned_rotations).magnitude())


def metric_stats(values: np.ndarray, prefix: str) -> dict[str, float]:
    if len(values) == 0:
        return {
            f"{prefix}_rmse": math.nan,
            f"{prefix}_mean": math.nan,
            f"{prefix}_median": math.nan,
            f"{prefix}_std": math.nan,
            f"{prefix}_p95": math.nan,
            f"{prefix}_max": math.nan,
            f"{prefix}_final": math.nan,
        }
    return {
        f"{prefix}_rmse": float(np.sqrt(np.mean(values**2))),
        f"{prefix}_mean": float(np.mean(values)),
        f"{prefix}_median": float(np.median(values)),
        f"{prefix}_std": float(np.std(values)),
        f"{prefix}_p95": float(np.percentile(values, 95)),
        f"{prefix}_max": float(np.max(values)),
        f"{prefix}_final": float(values[-1]),
    }


def delta_label(value: float | int) -> str:
    if float(value).is_integer():
        return str(int(value))
    return f"{float(value):.3f}".rstrip("0").rstrip(".").replace(".", "p")


def trans_rot_empty_stats(prefix: str) -> dict[str, float | int]:
    return {
        f"{prefix}_pairs": 0,
        f"{prefix}_trans_rmse_m": math.nan,
        f"{prefix}_trans_mean_m": math.nan,
        f"{prefix}_trans_median_m": math.nan,
        f"{prefix}_trans_p95_m": math.nan,
        f"{prefix}_trans_max_m": math.nan,
        f"{prefix}_trans_final_m": math.nan,
        f"{prefix}_rot_rmse_deg": math.nan,
        f"{prefix}_rot_mean_deg": math.nan,
        f"{prefix}_rot_median_deg": math.nan,
        f"{prefix}_rot_p95_deg": math.nan,
        f"{prefix}_rot_max_deg": math.nan,
        f"{prefix}_rot_final_deg": math.nan,
    }


def trans_rot_stats(
    trans_errors: np.ndarray,
    rot_errors: np.ndarray,
    prefix: str,
) -> dict[str, float | int]:
    if len(trans_errors) == 0 or len(rot_errors) == 0:
        return trans_rot_empty_stats(prefix)
    return {
        f"{prefix}_pairs": int(len(trans_errors)),
        f"{prefix}_trans_rmse_m": float(np.sqrt(np.mean(trans_errors**2))),
        f"{prefix}_trans_mean_m": float(np.mean(trans_errors)),
        f"{prefix}_trans_median_m": float(np.median(trans_errors)),
        f"{prefix}_trans_p95_m": float(np.percentile(trans_errors, 95)),
        f"{prefix}_trans_max_m": float(np.max(trans_errors)),
        f"{prefix}_trans_final_m": float(trans_errors[-1]),
        f"{prefix}_rot_rmse_deg": float(np.sqrt(np.mean(rot_errors**2))),
        f"{prefix}_rot_mean_deg": float(np.mean(rot_errors)),
        f"{prefix}_rot_median_deg": float(np.median(rot_errors)),
        f"{prefix}_rot_p95_deg": float(np.percentile(rot_errors, 95)),
        f"{prefix}_rot_max_deg": float(np.max(rot_errors)),
        f"{prefix}_rot_final_deg": float(rot_errors[-1]),
    }


def relative_pose_errors(
    start_positions: np.ndarray,
    start_rotations: Rotation,
    end_positions: np.ndarray,
    end_rotations: Rotation,
    gt_start_positions: np.ndarray,
    gt_start_rotations: Rotation,
    gt_end_positions: np.ndarray,
    gt_end_rotations: Rotation,
) -> tuple[np.ndarray, np.ndarray]:
    trans_errors: list[float] = []
    rot_errors: list[float] = []
    for idx in range(len(start_positions)):
        graph_rel_rot = start_rotations[idx].inv() * end_rotations[idx]
        gt_rel_rot = gt_start_rotations[idx].inv() * gt_end_rotations[idx]
        graph_rel_t = start_rotations[idx].inv().apply(
            end_positions[idx] - start_positions[idx]
        )
        gt_rel_t = gt_start_rotations[idx].inv().apply(
            gt_end_positions[idx] - gt_start_positions[idx]
        )
        trans_errors.append(float(np.linalg.norm(graph_rel_t - gt_rel_t)))
        rot_errors.append(
            float(np.degrees((gt_rel_rot.inv() * graph_rel_rot).magnitude()))
        )
    return np.asarray(trans_errors), np.asarray(rot_errors)


def rpe_metrics(
    positions: np.ndarray,
    rotations: Rotation,
    gt_positions: np.ndarray,
    gt_rotations: Rotation,
    delta: int,
    prefix: str = "rpe",
) -> dict[str, float]:
    if len(positions) <= delta or delta <= 0:
        return trans_rot_empty_stats(prefix)

    start_idx = np.arange(0, len(positions) - delta)
    end_idx = start_idx + delta
    trans, rot = relative_pose_errors(
        positions[start_idx],
        rotations[start_idx],
        positions[end_idx],
        rotations[end_idx],
        gt_positions[start_idx],
        gt_rotations[start_idx],
        gt_positions[end_idx],
        gt_rotations[end_idx],
    )
    return trans_rot_stats(trans, rot, prefix)


def rpe_distance_metrics(
    positions: np.ndarray,
    rotations: Rotation,
    gt_positions: np.ndarray,
    gt_rotations: Rotation,
    delta_m: float,
    prefix: str,
) -> dict[str, float]:
    gt_axis_raw = cumulative_distance(gt_positions)
    if len(gt_axis_raw) < 2 or gt_axis_raw[-1] <= delta_m or delta_m <= 0:
        return trans_rot_empty_stats(prefix)

    graph_axis, graph_pos, graph_quats, _ = unique_axis_samples(
        gt_axis_raw,
        positions,
        rotations.as_quat(),
    )
    gt_axis, gt_pos, gt_quats, _ = unique_axis_samples(
        gt_axis_raw,
        gt_positions,
        gt_rotations.as_quat(),
    )
    max_start = min(graph_axis[-1], gt_axis[-1]) - delta_m
    if max_start <= 0:
        return trans_rot_empty_stats(prefix)

    start_axis = gt_axis[gt_axis <= max_start]
    if len(start_axis) == 0:
        return trans_rot_empty_stats(prefix)
    end_axis = start_axis + delta_m

    graph_start_pos = interpolate_positions(graph_axis, graph_pos, start_axis)
    graph_end_pos = interpolate_positions(graph_axis, graph_pos, end_axis)
    gt_start_pos = interpolate_positions(gt_axis, gt_pos, start_axis)
    gt_end_pos = interpolate_positions(gt_axis, gt_pos, end_axis)
    graph_start_rot = interpolate_rotations(graph_axis, graph_quats, start_axis)
    graph_end_rot = interpolate_rotations(graph_axis, graph_quats, end_axis)
    gt_start_rot = interpolate_rotations(gt_axis, gt_quats, start_axis)
    gt_end_rot = interpolate_rotations(gt_axis, gt_quats, end_axis)

    trans, rot = relative_pose_errors(
        graph_start_pos,
        graph_start_rot,
        graph_end_pos,
        graph_end_rot,
        gt_start_pos,
        gt_start_rot,
        gt_end_pos,
        gt_end_rot,
    )
    return trans_rot_stats(trans, rot, prefix)


def all_rpe_metrics(
    positions: np.ndarray,
    rotations: Rotation,
    gt_positions: np.ndarray,
    gt_rotations: Rotation,
    keyframe_deltas: list[int],
    distance_deltas_m: list[float],
) -> dict[str, float]:
    out: dict[str, float] = {}
    primary_delta = keyframe_deltas[0] if keyframe_deltas else 1
    out.update(
        rpe_metrics(
            positions,
            rotations,
            gt_positions,
            gt_rotations,
            primary_delta,
            "rpe",
        )
    )
    for delta in keyframe_deltas:
        out.update(
            rpe_metrics(
                positions,
                rotations,
                gt_positions,
                gt_rotations,
                delta,
                f"rpe_kf_{delta}",
            )
        )
    for delta_m in distance_deltas_m:
        out.update(
            rpe_distance_metrics(
                positions,
                rotations,
                gt_positions,
                gt_rotations,
                delta_m,
                f"rpe_dist_{delta_label(delta_m)}m",
            )
        )
    return out


def trajectory_length(positions: np.ndarray) -> float:
    if len(positions) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))


def analyze_snapshot_robot(
    *,
    run_dir: Path,
    source: str,
    snapshot: pd.Series,
    robot_id: int,
    nodes: pd.DataFrame,
    edge_count_values: dict[str, int],
    gt: GroundTruthTrajectory,
    max_keyframe_id: int,
    alignment_mode: str,
    association: str,
    rpe_keyframe_deltas: list[int],
    rpe_distance_deltas_m: list[float],
) -> tuple[dict[str, object], dict[str, object], pd.DataFrame | None]:
    keyframe_ids, graph_positions, graph_rotations, graph_ros_time = prepare_graph_trajectory(
        nodes,
        robot_id,
    )
    if len(keyframe_ids) < 1:
        raise ValueError("No nodes for robot")

    used_association = association_used(graph_ros_time, keyframe_ids, association)
    gt_positions, gt_rotations, gt_times, gt_progress = matched_ground_truth(
        gt,
        keyframe_ids,
        graph_positions,
        graph_ros_time,
        max_keyframe_id,
        association,
    )
    alignment = align_trajectories(
        graph_positions,
        graph_rotations,
        gt_positions,
        gt_rotations,
        alignment_mode,
    )
    aligned_positions, aligned_rotations = alignment.apply(
        graph_positions,
        graph_rotations,
    )

    ate_values = np.linalg.norm(aligned_positions - gt_positions, axis=1)
    orient_values = rotation_errors_deg(aligned_rotations, gt_rotations)
    graph_length = trajectory_length(graph_positions)
    gt_length = trajectory_length(gt_positions)
    drift_final_m = float(ate_values[-1]) if len(ate_values) else math.nan
    drift_percent = (
        drift_final_m / gt_length * 100.0 if gt_length > 1e-12 else math.nan
    )

    row: dict[str, object] = {
        "source": source,
        "snapshot_seq": int(snapshot["snapshot_seq"]),
        "wall_time_unix_sec": float(snapshot["wall_time_unix_sec"]),
        "source_type": snapshot.get("source_type", ""),
        "reason": snapshot.get("reason", ""),
        "source_robot_id": snapshot.get("robot_id", ""),
        "origin_robot_id": snapshot.get("origin_robot_id", ""),
        "robot_id": robot_id,
        "robot_name": gt.robot_name,
        "association_used": used_association,
        "node_count_robot": int(len(keyframe_ids)),
        "node_count_total": int(snapshot["node_count"]),
        "edge_count_total": int(snapshot["edge_count"]),
        "first_keyframe_id": int(keyframe_ids[0]),
        "last_keyframe_id": int(keyframe_ids[-1]),
        "max_keyframe_id_source_robot": int(max_keyframe_id),
        "graph_start_ros_time": first_finite(graph_ros_time),
        "graph_end_ros_time": last_finite(graph_ros_time),
        "gt_start_wall_time": float(gt_times[0]),
        "gt_end_wall_time": float(gt_times[-1]),
        "gt_progress_start": float(gt_progress[0]),
        "gt_progress_end": float(gt_progress[-1]),
        "graph_path_length_m": graph_length,
        "gt_matched_path_length_m": gt_length,
        "gt_total_path_length_m": float(gt.cumulative_distance[-1]),
        "coverage_fraction": float(gt_progress[-1] - gt_progress[0]),
        "scale_ratio_graph_to_gt": (
            graph_length / gt_length if gt_length > 1e-12 else math.nan
        ),
        "drift_final_m": drift_final_m,
        "drift_percent": drift_percent,
        **edge_count_values,
        **metric_stats(ate_values, "ate_m"),
        **metric_stats(orient_values, "orientation_error_deg"),
        **all_rpe_metrics(
            aligned_positions,
            aligned_rotations,
            gt_positions,
            gt_rotations,
            rpe_keyframe_deltas,
            rpe_distance_deltas_m,
        ),
    }

    transform = alignment.matrix()
    transform_row: dict[str, object] = {
        "source": source,
        "snapshot_seq": int(snapshot["snapshot_seq"]),
        "wall_time_unix_sec": float(snapshot["wall_time_unix_sec"]),
        "robot_id": robot_id,
        "alignment": alignment_mode,
    }
    for ridx in range(4):
        for cidx in range(4):
            transform_row[f"T_{ridx}{cidx}"] = float(transform[ridx, cidx])

    point_errors = None
    if int(snapshot["snapshot_seq"]) == int(snapshot.get("_final_seq_for_source_robot", -1)):
        point_errors = pd.DataFrame(
            {
                "source": source,
                "robot_id": robot_id,
                "association_used": used_association,
                "keyframe_id": keyframe_ids,
                "graph_ros_time": (
                    graph_ros_time if graph_ros_time is not None else np.full(len(keyframe_ids), np.nan)
                ),
                "gt_progress": gt_progress,
                "graph_x": graph_positions[:, 0],
                "graph_y": graph_positions[:, 1],
                "graph_z": graph_positions[:, 2],
                "aligned_graph_x": aligned_positions[:, 0],
                "aligned_graph_y": aligned_positions[:, 1],
                "aligned_graph_z": aligned_positions[:, 2],
                "gt_x": gt_positions[:, 0],
                "gt_y": gt_positions[:, 1],
                "gt_z": gt_positions[:, 2],
                "ate_m": ate_values,
                "orientation_error_deg": orient_values,
            }
        )

    return row, transform_row, point_errors


def add_time_and_deltas(metrics: pd.DataFrame) -> pd.DataFrame:
    if metrics.empty:
        return metrics
    metrics = metrics.sort_values(["source", "robot_id", "snapshot_seq"]).copy()
    first_time = float(metrics["wall_time_unix_sec"].min())
    metrics["time_rel_sec"] = metrics["wall_time_unix_sec"] - first_time
    delta_columns = [
        "intra_loop_closures",
        "inter_loop_closures",
        "valid_inter_robot_loop_closures",
        "graph_inter_loop_closures",
        "graph_valid_inter_robot_loop_closures",
        "loop_edges",
        "inter_robot_edges",
        "odom_edges",
        "node_count_robot",
    ]
    for column in delta_columns:
        metrics[f"new_{column}"] = (
            metrics.groupby(["source", "robot_id"])[column]
            .diff()
            .fillna(metrics[column])
            .clip(lower=0)
            .astype(int)
        )
    metrics["new_total_loop_closures"] = (
        metrics["new_intra_loop_closures"] + metrics["new_inter_loop_closures"]
    )
    metrics["total_loop_closures"] = (
        metrics["intra_loop_closures"] + metrics["inter_loop_closures"]
    )
    return metrics


def rankdata(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values)
    ranks = np.empty(len(values), dtype=float)
    sorted_values = values[order]
    start = 0
    while start < len(values):
        end = start + 1
        while end < len(values) and sorted_values[end] == sorted_values[start]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + end - 1) + 1.0
        start = end
    return ranks


def pearson_corr(x_values: np.ndarray, y_values: np.ndarray) -> float:
    mask = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[mask]
    y_values = y_values[mask]
    if len(x_values) < 2:
        return math.nan
    if np.std(x_values) < 1e-12 or np.std(y_values) < 1e-12:
        return math.nan
    return float(np.corrcoef(x_values, y_values)[0, 1])


def slope(x_values: np.ndarray, y_values: np.ndarray) -> float:
    mask = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[mask]
    y_values = y_values[mask]
    if len(x_values) < 2 or np.var(x_values) < 1e-12:
        return math.nan
    return float(np.polyfit(x_values, y_values, 1)[0])


def correlations(metrics: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    variables = [
        "total_loop_closures",
        "intra_loop_closures",
        "inter_loop_closures",
        "valid_inter_robot_loop_closures",
        "inter_robot_loop_closure_precision",
        "inter_robot_loop_closure_recall",
        "graph_valid_inter_robot_loop_closures",
        "graph_inter_robot_loop_closure_precision",
        "graph_inter_robot_loop_closure_recall",
        "graph_map_consistency_trans_rmse_m",
        "graph_correction_trans_rmse_m",
        "inter_robot_edges",
        "new_total_loop_closures",
        "node_count_robot",
        "coverage_fraction",
    ]
    for (source, robot_id), group in metrics.groupby(["source", "robot_id"]):
        y_values = group["ate_m_rmse"].to_numpy(dtype=float)
        for variable in variables:
            x_values = group[variable].to_numpy(dtype=float)
            rows.append(
                {
                    "source": source,
                    "robot_id": robot_id,
                    "variable": variable,
                    "n": int(len(group)),
                    "pearson_corr_with_ate_rmse": pearson_corr(x_values, y_values),
                    "spearman_corr_with_ate_rmse": pearson_corr(
                        rankdata(x_values),
                        rankdata(y_values),
                    ),
                    "linear_slope_ate_rmse_per_unit": slope(x_values, y_values),
                }
            )
    return pd.DataFrame(rows)


def loop_events(metrics: pd.DataFrame, event_window: int) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for (source, robot_id), group in metrics.groupby(["source", "robot_id"]):
        group = group.sort_values("snapshot_seq").reset_index(drop=True)
        for idx, row in group.iterrows():
            new_intra = int(row["new_intra_loop_closures"])
            new_inter = int(row["new_inter_loop_closures"])
            if new_intra + new_inter <= 0:
                continue
            new_valid_inter = int(row.get("new_valid_inter_robot_loop_closures", 0))
            prev = group.iloc[idx - 1] if idx > 0 else None
            next_idx = min(idx + 1, len(group) - 1)
            window_idx = min(idx + event_window, len(group) - 1)
            before = float(prev["ate_m_rmse"]) if prev is not None else math.nan
            current = float(row["ate_m_rmse"])
            after_one = float(group.iloc[next_idx]["ate_m_rmse"])
            after_window = float(group.iloc[window_idx]["ate_m_rmse"])
            before_consistency = (
                float(prev["graph_map_consistency_trans_rmse_m"])
                if prev is not None and "graph_map_consistency_trans_rmse_m" in prev
                else math.nan
            )
            current_consistency = float(
                row.get("graph_map_consistency_trans_rmse_m", math.nan)
            )
            rows.append(
                {
                    "source": source,
                    "robot_id": robot_id,
                    "snapshot_seq": int(row["snapshot_seq"]),
                    "time_rel_sec": float(row["time_rel_sec"]),
                    "new_intra_loop_closures": new_intra,
                    "new_inter_loop_closures": new_inter,
                    "new_valid_inter_robot_loop_closures": new_valid_inter,
                    "new_total_loop_closures": new_intra + new_inter,
                    "ate_rmse_before_m": before,
                    "ate_rmse_current_m": current,
                    "ate_rmse_after_1_snapshot_m": after_one,
                    f"ate_rmse_after_{event_window}_snapshots_m": after_window,
                    "graph_correction_trans_rmse_m": float(
                        row.get("graph_correction_trans_rmse_m", math.nan)
                    ),
                    "graph_correction_rot_rmse_deg": float(
                        row.get("graph_correction_rot_rmse_deg", math.nan)
                    ),
                    "map_consistency_before_m": before_consistency,
                    "map_consistency_current_m": current_consistency,
                    "map_consistency_delta_current_minus_before_m": (
                        current_consistency - before_consistency
                        if math.isfinite(before_consistency)
                        and math.isfinite(current_consistency)
                        else math.nan
                    ),
                    "ate_delta_current_minus_before_m": (
                        current - before if math.isfinite(before) else math.nan
                    ),
                    "ate_delta_after_1_minus_current_m": after_one - current,
                    f"ate_delta_after_{event_window}_minus_before_m": (
                        after_window - before if math.isfinite(before) else math.nan
                    ),
                }
            )
    return pd.DataFrame(rows)


def final_summary(metrics: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for (source, robot_id), group in metrics.groupby(["source", "robot_id"]):
        group = group.sort_values("snapshot_seq").reset_index(drop=True)
        final = group.iloc[-1].to_dict()
        best_idx = group["ate_m_rmse"].idxmin()
        worst_idx = group["ate_m_rmse"].idxmax()
        inter_events = group.index[group["new_inter_loop_closures"] > 0].to_list()
        if not inter_events:
            inter_events = group.index[group["inter_loop_closures"] > 0].to_list()
        if inter_events:
            first_inter_idx = int(inter_events[0])
            before_idx = max(0, first_inter_idx - 1)
            ate_before_inter = float(group.loc[before_idx, "ate_m_rmse"])
            ate_at_inter = float(group.loc[first_inter_idx, "ate_m_rmse"])
            ate_final = float(group.iloc[-1]["ate_m_rmse"])
            ate_reduction_after_rendezvous = ate_before_inter - ate_final
            ate_reduction_after_rendezvous_percent = (
                ate_reduction_after_rendezvous / ate_before_inter * 100.0
                if abs(ate_before_inter) > 1e-12
                else math.nan
            )
            first_inter_snapshot = int(group.loc[first_inter_idx, "snapshot_seq"])
        else:
            first_inter_snapshot = -1
            ate_before_inter = math.nan
            ate_at_inter = math.nan
            ate_reduction_after_rendezvous = math.nan
            ate_reduction_after_rendezvous_percent = math.nan
        final.update(
            {
                "final_snapshot_seq": int(group.iloc[-1]["snapshot_seq"]),
                "best_ate_snapshot_seq": int(group.loc[best_idx, "snapshot_seq"]),
                "best_ate_rmse_m": float(group.loc[best_idx, "ate_m_rmse"]),
                "worst_ate_snapshot_seq": int(group.loc[worst_idx, "snapshot_seq"]),
                "worst_ate_rmse_m": float(group.loc[worst_idx, "ate_m_rmse"]),
                "ate_rmse_change_first_to_final_m": float(
                    group.iloc[-1]["ate_m_rmse"] - group.iloc[0]["ate_m_rmse"]
                ),
                "first_inter_robot_loop_snapshot_seq": first_inter_snapshot,
                "ate_rmse_before_first_inter_loop_m": ate_before_inter,
                "ate_rmse_at_first_inter_loop_m": ate_at_inter,
                "ate_reduction_after_rendezvous_m": ate_reduction_after_rendezvous,
                "ate_reduction_after_rendezvous_percent": ate_reduction_after_rendezvous_percent,
                "snapshots_analyzed": int(len(group)),
            }
        )
        rows.append(final)
    return pd.DataFrame(rows)


def write_point_errors(
    point_errors: dict[tuple[str, int], pd.DataFrame],
    output_dir: Path,
) -> None:
    point_dir = output_dir / "point_errors"
    point_dir.mkdir(parents=True, exist_ok=True)
    for (source, robot_id), df in point_errors.items():
        df.to_csv(point_dir / f"{source}_r{robot_id}_final_point_errors.csv", index=False)


def estimate_position_alignment(
    source_positions: np.ndarray,
    target_positions: np.ndarray,
) -> Alignment:
    mask = np.isfinite(source_positions).all(axis=1) & np.isfinite(target_positions).all(axis=1)
    source_positions = source_positions[mask]
    target_positions = target_positions[mask]
    if len(source_positions) == 0:
        return Alignment(Rotation.identity(), np.zeros(3))
    if len(source_positions) < 3:
        return Alignment(Rotation.identity(), target_positions[0] - source_positions[0])

    source_center = source_positions.mean(axis=0)
    target_center = target_positions.mean(axis=0)
    source_zero = source_positions - source_center
    target_zero = target_positions - target_center
    covariance = source_zero.T @ target_zero / len(source_positions)
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation_matrix = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation_matrix) < 0:
        vt_matrix[-1, :] *= -1.0
        rotation_matrix = vt_matrix.T @ u_matrix.T
    rotation = Rotation.from_matrix(rotation_matrix)
    translation = target_center - rotation.apply(source_center)
    return Alignment(rotation, translation)


def invert_alignment(alignment: Alignment) -> Alignment:
    inverse_rotation = alignment.rotation.inv()
    inverse_translation = -inverse_rotation.apply(alignment.translation)
    return Alignment(inverse_rotation, inverse_translation)


def dataframe_to_markdown(df: pd.DataFrame, floatfmt: str = ".4f") -> str:
    if df.empty:
        return ""

    def cell(value: object) -> str:
        if pd.isna(value):
            return ""
        if isinstance(value, (float, np.floating)):
            return format(float(value), floatfmt)
        if isinstance(value, (int, np.integer)):
            return str(int(value))
        return str(value)

    columns = [str(column) for column in df.columns]
    rows = [[cell(value) for value in row] for row in df.to_numpy()]
    widths = [
        max(len(columns[idx]), *(len(row[idx]) for row in rows))
        for idx in range(len(columns))
    ]
    header = "| " + " | ".join(
        columns[idx].ljust(widths[idx]) for idx in range(len(columns))
    ) + " |"
    divider = "| " + " | ".join("-" * widths[idx] for idx in range(len(columns))) + " |"
    body = [
        "| " + " | ".join(row[idx].ljust(widths[idx]) for idx in range(len(columns))) + " |"
        for row in rows
    ]
    return "\n".join([header, divider, *body])


def load_communication_summary(run_dir: Path) -> dict[str, object] | None:
    summary_path = run_dir / "communication" / "communication_summary.json"
    if not summary_path.is_file():
        return None
    try:
        return json.loads(summary_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(
            f"Warning: could not read communication summary {summary_path}: {exc}",
            file=sys.stderr,
        )
        return None


def load_communication_topics(run_dir: Path) -> pd.DataFrame:
    topics_path = run_dir / "communication" / "communication_topics.csv"
    if not topics_path.is_file():
        return pd.DataFrame()
    try:
        df = pd.read_csv(topics_path)
    except Exception as exc:
        print(
            f"Warning: could not read communication topics {topics_path}: {exc}",
            file=sys.stderr,
        )
        return pd.DataFrame()
    for column in [
        "message_count",
        "total_bytes",
        "total_MB",
        "average_message_size_bytes",
        "average_bandwidth_Bps",
        "peak_bandwidth_Bps",
    ]:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")
    return df


def load_communication_robot_topics(run_dir: Path) -> pd.DataFrame:
    topics_path = run_dir / "communication" / "communication_robot_topics.csv"
    if not topics_path.is_file():
        return pd.DataFrame()
    try:
        df = pd.read_csv(topics_path)
    except Exception as exc:
        print(
            f"Warning: could not read communication robot topics {topics_path}: {exc}",
            file=sys.stderr,
        )
        return pd.DataFrame()
    for column in [
        "message_count",
        "total_bytes",
        "total_MB",
        "average_message_size_bytes",
        "average_bandwidth_Bps",
        "peak_bandwidth_Bps",
    ]:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")
    return df


def load_communication_timeseries(run_dir: Path) -> pd.DataFrame:
    timeseries_path = run_dir / "communication" / "communication_timeseries.csv"
    if not timeseries_path.is_file():
        return pd.DataFrame()
    try:
        df = pd.read_csv(timeseries_path)
    except Exception as exc:
        print(
            f"Warning: could not read communication timeseries {timeseries_path}: {exc}",
            file=sys.stderr,
        )
        return pd.DataFrame()
    for column in ["timestamp", "messages", "bytes", "bandwidth_Bps"]:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")
    for column in ["traffic_class", "source_robot", "peer_robot"]:
        if column not in df.columns:
            df[column] = "unknown"
    return df.dropna(subset=["timestamp", "bandwidth_Bps"])


def load_communication_services(run_dir: Path) -> pd.DataFrame:
    service_paths = sorted((run_dir / "communication").glob("communication_services_*.csv"))
    if not service_paths:
        return pd.DataFrame()

    frames: list[pd.DataFrame] = []
    for path in service_paths:
        try:
            frame = pd.read_csv(path)
        except Exception as exc:
            print(
                f"Warning: could not read communication services {path}: {exc}",
                file=sys.stderr,
            )
            continue
        if frame.empty:
            continue
        frame["metrics_file"] = path.name
        frames.append(frame)

    if not frames:
        return pd.DataFrame()

    df = pd.concat(frames, ignore_index=True)
    for column in [
        "timestamp_sec",
        "request_bytes",
        "response_bytes",
        "total_bytes",
        "response_keyframes",
        "response_edges",
        "response_cloud_bytes",
        "request_processed_keyframes",
        "request_processed_edges",
    ]:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce").fillna(0)
    if "total_bytes" in df.columns:
        fallback_total = df.get("request_bytes", 0) + df.get("response_bytes", 0)
        df["total_bytes"] = df["total_bytes"].where(df["total_bytes"] > 0, fallback_total)
    return df.dropna(subset=["timestamp_sec"])


def make_communication_plots(run_dir: Path, output_dir: Path) -> None:
    communication = load_communication_timeseries(run_dir)
    services = load_communication_services(run_dir)
    if communication.empty and services.empty:
        return

    try:
        import warnings

        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message="Unable to import Axes3D.*",
                category=UserWarning,
            )
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
    except Exception:
        print(
            "matplotlib is not available; skipping communication plots.",
            file=sys.stderr,
        )
        return

    plot_dir = output_dir / "plots" / "communication"
    plot_dir.mkdir(parents=True, exist_ok=True)
    for path in plot_dir.glob("*.png"):
        path.unlink()

    readme = """# Communication Plots

These plots show estimated logical inter-robot SLAM communication.

- X axis: minutes from the first recorded communication sample.
- Y axis: serialized ROS message payload bandwidth in MB/s.
- `bandwidth_over_time.png` sums measured communication topics per recorder sample window.
- `bandwidth_by_source_robot.png` appears when the recorder saved source_robot attribution. It uses a stacked layout with total bandwidth on top and one panel per source robot below.
- `total_mb_by_source_robot.png` appears when robot-level communication totals are available.
- `service_graph_bandwidth_by_robot.png` appears when MRG graph service CSVs exist. It groups graph service payload bytes into 1-second ROS-time buckets and shows one panel per robot involved in graph exchanges, so graph exchanges appear as spikes.
- In the service plot, the total panel counts each service event once; robot panels count an event for each participating robot.
- This is not physical network bandwidth; DDS transport effects are outside this metric.
"""
    (plot_dir / "README.md").write_text(readme, encoding="utf-8")

    if not communication.empty:
        communication = communication.sort_values("timestamp")
        first_timestamp = float(communication["timestamp"].min())
        total = (
            communication.groupby("timestamp", as_index=False)["bandwidth_Bps"]
            .sum()
            .sort_values("timestamp")
        )
        topic_totals = (
            communication.groupby("topic")["bytes"].sum().sort_values(ascending=False)
        )
        top_topics = list(topic_totals.head(5).index)
        topic_series = (
            communication.groupby(["timestamp", "topic"], as_index=False)["bandwidth_Bps"]
            .sum()
            .sort_values("timestamp")
        )

        fig, axis = plt.subplots(figsize=(11, 5.5))
        total_time_min = (
            total["timestamp"].to_numpy(dtype=float) - first_timestamp
        ) / 60.0
        axis.plot(
            total_time_min,
            total["bandwidth_Bps"].to_numpy(dtype=float) / 1_000_000.0,
            label="Total",
            linewidth=2.2,
            color="black",
        )
        for topic in top_topics:
            topic_group = topic_series[topic_series["topic"] == topic]
            topic_time_min = (
                topic_group["timestamp"].to_numpy(dtype=float) - first_timestamp
            ) / 60.0
            axis.plot(
                topic_time_min,
                topic_group["bandwidth_Bps"].to_numpy(dtype=float) / 1_000_000.0,
                linewidth=1.3,
                alpha=0.8,
                label=topic,
            )
        axis.set_xlabel("Time from first communication sample [min]")
        axis.set_ylabel("Estimated logical bandwidth [MB/s]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize="small")
        axis.set_title("Estimated logical inter-robot SLAM communication")
        fig.tight_layout()
        fig.savefig(plot_dir / "bandwidth_over_time.png", dpi=160)
        plt.close(fig)

        if "source_robot" in communication.columns:
            known_sources = communication[
                ~communication["source_robot"].astype(str).isin(
                    ["", "unknown", "broadcast", "multiple"]
                )
            ].copy()
            if not known_sources.empty:
                robot_series = (
                    known_sources.groupby(
                        ["timestamp", "source_robot"],
                        as_index=False,
                    )["bandwidth_Bps"]
                    .sum()
                    .sort_values("timestamp")
                )
                robots = sorted(robot_series["source_robot"].astype(str).unique())
                fig, axes = plt.subplots(
                    len(robots) + 1,
                    1,
                    figsize=(11, max(5.5, 1.9 * (len(robots) + 1))),
                    sharex=True,
                )
                axes = np.atleast_1d(axes)

                total_by_source = (
                    robot_series.groupby("timestamp", as_index=False)["bandwidth_Bps"]
                    .sum()
                    .sort_values("timestamp")
                )
                total_time_min = (
                    total_by_source["timestamp"].to_numpy(dtype=float)
                    - first_timestamp
                ) / 60.0
                axes[0].plot(
                    total_time_min,
                    total_by_source["bandwidth_Bps"].to_numpy(dtype=float)
                    / 1_000_000.0,
                    linewidth=2.2,
                    color="black",
                    label="Total source robots",
                )
                axes[0].set_ylabel("Total\n[MB/s]")
                axes[0].grid(True, alpha=0.25)
                axes[0].legend(loc="upper right", fontsize="small")

                color_cycle = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
                robot_colors = {
                    robot: color_cycle[index % len(color_cycle)] if color_cycle else None
                    for index, robot in enumerate(robots)
                }
                for robot, axis in zip(robots, axes[1:]):
                    robot_group = robot_series[
                        robot_series["source_robot"].astype(str) == robot
                    ]
                    robot_time_min = (
                        robot_group["timestamp"].to_numpy(dtype=float)
                        - first_timestamp
                    ) / 60.0
                    axis.plot(
                        robot_time_min,
                        robot_group["bandwidth_Bps"].to_numpy(dtype=float)
                        / 1_000_000.0,
                        linewidth=1.5,
                        alpha=0.9,
                        color=robot_colors[robot],
                        label=robot,
                    )
                    axis.set_ylabel(f"{robot}\n[MB/s]")
                    axis.grid(True, alpha=0.25)
                    axis.legend(loc="upper right", fontsize="small")
                axes[-1].set_xlabel("Time from first communication sample [min]")
                fig.suptitle("Estimated communication bandwidth by source robot")
                fig.tight_layout()
                fig.savefig(plot_dir / "bandwidth_by_source_robot.png", dpi=160)
                plt.close(fig)

    robot_topics = load_communication_robot_topics(run_dir)
    if not robot_topics.empty and {"source_robot", "total_bytes"}.issubset(
        robot_topics.columns
    ):
        source_totals = (
            robot_topics[
                ~robot_topics["source_robot"].astype(str).isin(
                    ["", "unknown", "broadcast", "multiple"]
                )
            ]
            .groupby("source_robot", as_index=False)["total_bytes"]
            .sum()
            .sort_values("total_bytes", ascending=False)
        )
        if not source_totals.empty:
            fig, axis = plt.subplots(figsize=(8.5, 4.8))
            axis.bar(
                source_totals["source_robot"].astype(str),
                source_totals["total_bytes"].to_numpy(dtype=float) / 1_000_000.0,
            )
            axis.set_xlabel("Source robot")
            axis.set_ylabel("Estimated TX total [MB]")
            axis.grid(True, axis="y", alpha=0.25)
            axis.set_title("Estimated communication total by source robot")
            fig.tight_layout()
            fig.savefig(plot_dir / "total_mb_by_source_robot.png", dpi=160)
            plt.close(fig)

    if not services.empty and {"timestamp_sec", "source_robot", "total_bytes"}.issubset(
        services.columns
    ):
        bucket_sec = 1.0
        services = services.sort_values("timestamp_sec").copy()
        first_service_time = float(services["timestamp_sec"].min())
        min_bucket = math.floor(first_service_time / bucket_sec) * bucket_sec
        max_bucket = math.ceil(float(services["timestamp_sec"].max()) / bucket_sec) * bucket_sec
        services["bucket_sec"] = (
            np.floor(services["timestamp_sec"].to_numpy(dtype=float) / bucket_sec)
            * bucket_sec
        )
        participant_frames = []
        source_bytes = services[["bucket_sec", "source_robot", "total_bytes"]].rename(
            columns={"source_robot": "robot"}
        )
        participant_frames.append(source_bytes)
        if "peer_robot" in services.columns:
            peer_bytes = services[["bucket_sec", "peer_robot", "total_bytes"]].rename(
                columns={"peer_robot": "robot"}
            )
            participant_frames.append(peer_bytes)
        robot_services = pd.concat(participant_frames, ignore_index=True)
        robot_services = robot_services.dropna(subset=["robot"])
        robot_services = robot_services[robot_services["robot"].astype(str) != ""]
        per_robot = (
            robot_services.groupby(["bucket_sec", "robot"], as_index=False)["total_bytes"]
            .sum()
            .sort_values("bucket_sec")
        )
        buckets = np.arange(min_bucket, max_bucket + bucket_sec, bucket_sec)

        total_by_bucket = (
            services.groupby("bucket_sec")["total_bytes"].sum().reindex(buckets, fill_value=0)
        )
        time_min = (buckets - first_service_time) / 60.0
        robots = sorted(per_robot["robot"].dropna().astype(str).unique())
        fig, axes = plt.subplots(
            len(robots) + 1,
            1,
            figsize=(11, max(5.5, 1.9 * (len(robots) + 1))),
            sharex=True,
        )
        axes = np.atleast_1d(axes)
        total_axis = axes[0]
        total_axis.step(
            time_min,
            total_by_bucket.to_numpy(dtype=float) / bucket_sec / 1_000_000.0,
            where="post",
            label="Total graph services",
            linewidth=2.2,
            color="black",
        )
        total_axis.set_ylabel("Total\n[MB/s]")
        total_axis.grid(True, alpha=0.25)
        total_axis.legend(loc="upper right", fontsize="small")

        color_cycle = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
        robot_colors = {
            robot: color_cycle[index % len(color_cycle)] if color_cycle else None
            for index, robot in enumerate(robots)
        }
        for robot, axis in zip(robots, axes[1:]):
            robot_bytes = (
                per_robot[per_robot["robot"].astype(str) == robot]
                .set_index("bucket_sec")["total_bytes"]
                .reindex(buckets, fill_value=0)
            )
            axis.step(
                time_min,
                robot_bytes.to_numpy(dtype=float) / bucket_sec / 1_000_000.0,
                where="post",
                linewidth=1.5,
                alpha=0.85,
                color=robot_colors[robot],
                label=robot,
            )
            axis.set_ylabel(f"{robot}\n[MB/s]")
            axis.grid(True, alpha=0.25)
            axis.legend(loc="upper right", fontsize="small")
        axes[-1].set_xlabel("Time from first graph service event [min, ROS time]")
        fig.suptitle("MRG graph service communication by robot")
        fig.tight_layout()
        fig.savefig(plot_dir / "service_graph_bandwidth_by_robot.png", dpi=160)
        plt.close(fig)


def make_plots(metrics: pd.DataFrame, point_errors: dict[tuple[str, int], pd.DataFrame], output_dir: Path) -> None:
    try:
        import warnings

        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message="Unable to import Axes3D.*",
                category=UserWarning,
            )
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
    except Exception:
        print("matplotlib is not available; skipping plots.", file=sys.stderr)
        return

    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    subdirs = {
        "ate_loops": plot_dir / "ate_loops",
        "loop_quality": plot_dir / "loop_quality",
        "rendezvous": plot_dir / "rendezvous",
        "graph_correction": plot_dir / "graph_correction",
        "rpe": plot_dir / "rpe",
        "trajectory": plot_dir / "trajectory",
        "final_ate": plot_dir / "final_ate",
    }
    for directory in subdirs.values():
        directory.mkdir(parents=True, exist_ok=True)
        for path in directory.glob("*.png"):
            path.unlink()

    stale_patterns = [
        "ate_loops_*.png",
        "ate_rmse_overview.png",
        "final_ate_by_keyframe_*.png",
        "rpe_*.png",
        "trajectory_*.png",
    ]
    for pattern in stale_patterns:
        for path in plot_dir.glob(pattern):
            if path.is_file():
                path.unlink()

    readmes = {
        "ate_loops": """# ATE Loops Plots

These plots show ATE RMSE over snapshot time.

- X axis: minutes from the first recorded pose-graph snapshot.
- Y axis: Absolute Trajectory Error RMSE in meters.
- Green vertical lines mark snapshots where new intra-robot loop closures appeared.
- Red vertical lines mark snapshots where new inter-robot loop closures appeared.
- The overview plot overlays all analyzed source/robot pairs for quick comparison.
""",
        "loop_quality": """# Loop Quality Plots

These plots show inter-robot loop-closure quality over time.

- X axis: minutes from the first recorded pose-graph snapshot.
- Top panel: ATE RMSE in meters.
- Bottom panel, left axis: cumulative accepted inter-robot loop closures, split into GT-valid and GT-invalid accepted closures.
- Bottom panel, right axis: inter-robot loop-closure precision, valid accepted closures divided by evaluated accepted closures.
- Green vertical lines mark snapshots where new GT-valid inter-robot loop closures appeared.
- Red vertical lines mark snapshots where accepted inter-robot loop closures appeared but did not pass the GT validation threshold.
""",
        "rendezvous": """# Rendezvous ATE Plots

These plots summarize the ATE around the first inter-robot loop closure event.

- X axis: rendezvous phase.
- Y axis: ATE RMSE in meters.
- For each robot, the bars compare the snapshot before the first inter-robot loop closure, the event snapshot, one snapshot after the event, and the final snapshot.
- A lower final bar indicates that the collaborative graph updates reduced global trajectory error after rendezvous.
""",
        "graph_correction": """# Graph Correction Plots

These plots show how much the optimized graph changed between consecutive snapshots.

- X axis: minutes from the first recorded pose-graph snapshot.
- Left Y axis: translational graph-correction RMSE in meters over keyframes shared by consecutive snapshots.
- Right Y axis: rotational graph-correction RMSE in degrees.
- Green/red vertical lines mark new intra/inter loop closure events.
- Peaks show where loop closures or graph updates moved already-existing keyframes.
""",
        "rpe": """# RPE Plots

These plots show local drift using Relative Pose Error.

- Keyframe plots use the primary keyframe delta shown in the report configuration.
- Distance plots compute RPE at approximately 1 m, 5 m, and 10 m along the matched ground-truth trajectory.
- X axis: minutes from the first recorded pose-graph snapshot.
- Y axes: translational RMSE in meters and rotational RMSE in degrees.
- Vertical lines mark new loop-closure events, making it easier to see whether local drift changes after graph updates.
""",
        "trajectory": """# Trajectory Plots

These plots compare the final SLAM trajectory with ground truth in the XY plane.

- X/Y axes: position in meters.
- Ground truth is plotted against the final aligned graph trajectory.
- Start/end markers show trajectory endpoints.
- Green markers on the SLAM trajectory indicate keyframes near intra-robot loop-closure events.
- Red markers indicate keyframes near inter-robot loop-closure events.

Files named `trajectory_<source>_global.png` show the final multi-robot graph
published by one robot-centric source. Solid lines are graph trajectories in the
source frame. Dashed lines are ground truth transformed into that same source
frame using the source robot trajectory as the reference alignment. Circle and
square markers show graph start and end points.
""",
        "final_ate": """# Final ATE Plots

These plots show per-keyframe ATE in the final graph.

- X axis: keyframe id.
- Y axis: Absolute Trajectory Error in meters for the final snapshot.
- Green/red vertical lines mark keyframes near intra/inter loop-closure events observed over the run.
- These plots help identify where along the trajectory the final graph still has local error peaks.
""",
    }
    for name, content in readmes.items():
        (subdirs[name] / "README.md").write_text(content, encoding="utf-8")

    def add_loop_event_lines(axis, group: pd.DataFrame, time_min: np.ndarray) -> None:
        intra_added = False
        inter_added = False
        for idx, row in group.reset_index(drop=True).iterrows():
            x_value = float(time_min[idx])
            if int(row["new_intra_loop_closures"]) > 0:
                axis.axvline(
                    x=x_value,
                    color="tab:green",
                    linestyle=":",
                    linewidth=1.0,
                    alpha=0.35,
                    label="New intra loop closure" if not intra_added else None,
                    zorder=0,
                )
                intra_added = True
            if int(row["new_inter_loop_closures"]) > 0:
                axis.axvline(
                    x=x_value,
                    color="tab:red",
                    linestyle="--",
                    linewidth=1.0,
                    alpha=0.35,
                    label="New inter loop closure" if not inter_added else None,
                    zorder=0,
                )
                inter_added = True

    def add_loop_keyframe_lines(axis, group: pd.DataFrame) -> None:
        intra_added = False
        inter_added = False
        for _, row in group.iterrows():
            x_value = float(row["last_keyframe_id"])
            if int(row["new_intra_loop_closures"]) > 0:
                axis.axvline(
                    x=x_value,
                    color="tab:green",
                    linestyle=":",
                    linewidth=1.0,
                    alpha=0.35,
                    label="New intra loop closure" if not intra_added else None,
                    zorder=0,
                )
                intra_added = True
            if int(row["new_inter_loop_closures"]) > 0:
                axis.axvline(
                    x=x_value,
                    color="tab:red",
                    linestyle="--",
                    linewidth=1.0,
                    alpha=0.35,
                    label="New inter loop closure" if not inter_added else None,
                    zorder=0,
                )
                inter_added = True

    def add_loop_quality_lines(axis, group: pd.DataFrame, time_min: np.ndarray) -> None:
        valid_added = False
        invalid_added = False
        for idx, row in group.reset_index(drop=True).iterrows():
            x_value = float(time_min[idx])
            new_valid = int(row.get("new_valid_inter_robot_loop_closures", 0))
            new_inter = int(row.get("new_inter_loop_closures", 0))
            new_invalid = max(0, new_inter - new_valid)
            if new_valid > 0:
                axis.axvline(
                    x=x_value,
                    color="tab:green",
                    linestyle="-",
                    linewidth=1.0,
                    alpha=0.35,
                    label="New valid inter loop" if not valid_added else None,
                    zorder=0,
                )
                valid_added = True
            if new_invalid > 0:
                axis.axvline(
                    x=x_value,
                    color="tab:red",
                    linestyle="--",
                    linewidth=1.0,
                    alpha=0.30,
                    label="New invalid inter loop" if not invalid_added else None,
                    zorder=0,
                )
                invalid_added = True

    def add_loop_markers_on_trajectory(axis, df: pd.DataFrame, group: pd.DataFrame) -> None:
        if df.empty or group is None or group.empty:
            return
        keyframes = df["keyframe_id"].to_numpy(dtype=float)
        intra_added = False
        inter_added = False
        for _, row in group.iterrows():
            if int(row["new_intra_loop_closures"]) <= 0 and int(row["new_inter_loop_closures"]) <= 0:
                continue
            event_keyframe = float(row["last_keyframe_id"])
            if len(keyframes) == 0:
                continue
            idx = int(np.argmin(np.abs(keyframes - event_keyframe)))
            x_value = float(df["aligned_graph_x"].iloc[idx])
            y_value = float(df["aligned_graph_y"].iloc[idx])
            if int(row["new_intra_loop_closures"]) > 0:
                axis.scatter(
                    x_value,
                    y_value,
                    marker="o",
                    s=34,
                    color="tab:green",
                    alpha=0.75,
                    label="Intra loop event" if not intra_added else None,
                    zorder=4,
                )
                intra_added = True
            if int(row["new_inter_loop_closures"]) > 0:
                axis.scatter(
                    x_value,
                    y_value,
                    marker="x",
                    s=48,
                    color="tab:red",
                    alpha=0.85,
                    label="Inter loop event" if not inter_added else None,
                    zorder=5,
                )
                inter_added = True

    def available_distance_rpe_columns(group: pd.DataFrame) -> list[tuple[str, str, str]]:
        out: list[tuple[str, str, str]] = []
        for column in group.columns:
            match = re.fullmatch(r"rpe_dist_(.+m)_trans_rmse_m", str(column))
            if match is None:
                continue
            label = match.group(1).replace("p", ".")
            rot_column = f"rpe_dist_{match.group(1)}_rot_rmse_deg"
            if rot_column in group.columns:
                out.append((label, column, rot_column))
        return sorted(out, key=lambda item: float(item[0].removesuffix("m")))

    metrics_by_source_robot = {
        key: group.sort_values("snapshot_seq").reset_index(drop=True)
        for key, group in metrics.groupby(["source", "robot_id"])
    }
    point_errors_by_source: dict[str, dict[int, pd.DataFrame]] = {}
    for (source, robot_id), df in point_errors.items():
        point_errors_by_source.setdefault(source, {})[robot_id] = df

    for (source, robot_id), group in metrics.groupby(["source", "robot_id"]):
        group = group.sort_values("snapshot_seq").reset_index(drop=True)
        time_min = group["time_rel_sec"].to_numpy(dtype=float) / 60.0

        fig, axis = plt.subplots(figsize=(11, 5.5))
        axis.plot(time_min, group["ate_m_rmse"], label="ATE RMSE [m]", linewidth=2)
        add_loop_event_lines(axis, group, time_min)
        axis.set_xlabel("Snapshot time from first graph [min]")
        axis.set_ylabel("ATE [m]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
        axis.set_title(f"{source} r{robot_id}: ATE vs loop closures")
        fig.tight_layout()
        fig.savefig(subdirs["ate_loops"] / f"ate_loops_{source}_r{robot_id}.png", dpi=160)
        plt.close(fig)

        fig, axes = plt.subplots(
            2,
            1,
            figsize=(11, 7.2),
            sharex=True,
            gridspec_kw={"height_ratios": [2.0, 1.4]},
        )
        axes[0].plot(
            time_min,
            group["ate_m_rmse"],
            label="ATE RMSE [m]",
            linewidth=2.0,
            color="tab:blue",
        )
        add_loop_quality_lines(axes[0], group, time_min)
        axes[0].set_ylabel("ATE RMSE [m]")
        axes[0].grid(True, alpha=0.25)
        axes[0].legend(loc="best")
        axes[0].set_title(f"{source} r{robot_id}: inter-robot loop quality")

        valid_counts = group.get(
            "valid_inter_robot_loop_closures",
            pd.Series(np.zeros(len(group)), index=group.index),
        ).to_numpy(dtype=float)
        invalid_counts = group.get(
            "invalid_inter_robot_loop_closures",
            pd.Series(np.zeros(len(group)), index=group.index),
        ).to_numpy(dtype=float)
        total_counts = valid_counts + invalid_counts
        axes[1].step(
            time_min,
            total_counts,
            where="post",
            color="0.35",
            linewidth=1.6,
            label="Accepted inter loops",
        )
        axes[1].step(
            time_min,
            valid_counts,
            where="post",
            color="tab:green",
            linewidth=1.8,
            label="GT-valid accepted",
        )
        axes[1].step(
            time_min,
            invalid_counts,
            where="post",
            color="tab:red",
            linewidth=1.6,
            label="GT-invalid accepted",
        )
        axes[1].set_xlabel("Snapshot time from first graph [min]")
        axes[1].set_ylabel("Closure count")
        axes[1].grid(True, alpha=0.25)

        precision_axis = axes[1].twinx()
        if "inter_robot_loop_closure_precision" in group.columns:
            precision = group["inter_robot_loop_closure_precision"].to_numpy(dtype=float)
            precision_axis.plot(
                time_min,
                precision,
                color="tab:purple",
                linewidth=1.5,
                label="Precision",
            )
            if np.isfinite(precision).any():
                precision_axis.set_ylim(-0.03, 1.03)
        precision_axis.set_ylabel("Precision")
        lines, labels = axes[1].get_legend_handles_labels()
        lines2, labels2 = precision_axis.get_legend_handles_labels()
        axes[1].legend(lines + lines2, labels + labels2, loc="best")
        fig.tight_layout()
        fig.savefig(
            subdirs["loop_quality"] / f"loop_quality_{source}_r{robot_id}.png",
            dpi=160,
        )
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(11, 5.5))
        axis.plot(time_min, group["rpe_trans_rmse_m"], label="RPE trans RMSE [m]")
        add_loop_event_lines(axis, group, time_min)
        axis.set_xlabel("Snapshot time from first graph [min]")
        axis.set_ylabel("RPE translation [m]")
        axis.grid(True, alpha=0.25)
        axis2 = axis.twinx()
        axis2.plot(
            time_min,
            group["rpe_rot_rmse_deg"],
            label="RPE rot RMSE [deg]",
            color="tab:orange",
        )
        axis2.set_ylabel("RPE rotation [deg]")
        lines, labels = axis.get_legend_handles_labels()
        lines2, labels2 = axis2.get_legend_handles_labels()
        axis.legend(lines + lines2, labels + labels2, loc="best")
        axis.set_title(f"{source} r{robot_id}: RPE evolution")
        fig.tight_layout()
        fig.savefig(subdirs["rpe"] / f"rpe_keyframe_{source}_r{robot_id}.png", dpi=160)
        plt.close(fig)

        distance_columns = available_distance_rpe_columns(group)
        if distance_columns:
            fig, axes = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
            for label, trans_col, rot_col in distance_columns:
                axes[0].plot(
                    time_min,
                    group[trans_col],
                    linewidth=1.5,
                    label=f"{label} trans",
                )
                axes[1].plot(
                    time_min,
                    group[rot_col],
                    linewidth=1.5,
                    label=f"{label} rot",
                )
            add_loop_event_lines(axes[0], group, time_min)
            add_loop_event_lines(axes[1], group, time_min)
            axes[0].set_ylabel("RPE trans RMSE [m]")
            axes[1].set_ylabel("RPE rot RMSE [deg]")
            axes[1].set_xlabel("Snapshot time from first graph [min]")
            for current_axis in axes:
                current_axis.grid(True, alpha=0.25)
                current_axis.legend(loc="best")
            axes[0].set_title(f"{source} r{robot_id}: distance-based RPE")
            fig.tight_layout()
            fig.savefig(
                subdirs["rpe"] / f"rpe_distance_{source}_r{robot_id}.png",
                dpi=160,
            )
            plt.close(fig)

        fig, axis = plt.subplots(figsize=(11, 5.5))
        if "graph_correction_trans_rmse_m" in group.columns:
            axis.plot(
                time_min,
                group["graph_correction_trans_rmse_m"],
                label="Graph correction trans RMSE [m]",
                linewidth=1.8,
            )
        add_loop_event_lines(axis, group, time_min)
        axis.set_xlabel("Snapshot time from first graph [min]")
        axis.set_ylabel("Graph correction translation [m]")
        axis.grid(True, alpha=0.25)
        axis2 = axis.twinx()
        if "graph_correction_rot_rmse_deg" in group.columns:
            axis2.plot(
                time_min,
                group["graph_correction_rot_rmse_deg"],
                label="Graph correction rot RMSE [deg]",
                color="tab:orange",
                linewidth=1.4,
            )
        axis2.set_ylabel("Graph correction rotation [deg]")
        lines, labels = axis.get_legend_handles_labels()
        lines2, labels2 = axis2.get_legend_handles_labels()
        axis.legend(lines + lines2, labels + labels2, loc="best")
        axis.set_title(f"{source} r{robot_id}: graph correction vs loop closures")
        fig.tight_layout()
        fig.savefig(
            subdirs["graph_correction"] / f"graph_correction_{source}_r{robot_id}.png",
            dpi=160,
        )
        plt.close(fig)

    for (source, robot_id), df in point_errors.items():
        fig, axis = plt.subplots(figsize=(7, 7))
        axis.plot(df["gt_x"], df["gt_y"], label="Ground truth", linewidth=2)
        axis.plot(
            df["aligned_graph_x"],
            df["aligned_graph_y"],
            label="Aligned graph",
            linewidth=1.5,
        )
        group = metrics_by_source_robot.get((source, robot_id))
        if group is not None:
            add_loop_markers_on_trajectory(axis, df, group)
        axis.scatter(df["gt_x"].iloc[0], df["gt_y"].iloc[0], label="Start", s=35)
        axis.scatter(df["gt_x"].iloc[-1], df["gt_y"].iloc[-1], label="End", s=35)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
        axis.set_title(f"{source} r{robot_id}: final trajectory")
        fig.tight_layout()
        fig.savefig(subdirs["trajectory"] / f"trajectory_{source}_r{robot_id}.png", dpi=160)
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(11, 4.8))
        axis.plot(df["keyframe_id"], df["ate_m"], linewidth=1.8, label="ATE [m]")
        group = metrics_by_source_robot.get((source, robot_id))
        if group is not None:
            add_loop_keyframe_lines(axis, group)
        axis.set_xlabel("Keyframe id")
        axis.set_ylabel("ATE [m]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
        axis.set_title(f"{source} r{robot_id}: final per-keyframe ATE")
        fig.tight_layout()
        fig.savefig(
            subdirs["final_ate"] / f"final_ate_by_keyframe_{source}_r{robot_id}.png",
            dpi=160,
        )
        plt.close(fig)

    def point_positions(df: pd.DataFrame, prefix: str) -> np.ndarray:
        return df[[f"{prefix}_x", f"{prefix}_y", f"{prefix}_z"]].to_numpy(dtype=float)

    for source, robot_dfs in sorted(point_errors_by_source.items()):
        if not robot_dfs:
            continue
        source_robot_id = robot_id_from_text(source)
        if source_robot_id not in robot_dfs:
            source_robot_id = sorted(robot_dfs)[0]
        reference_df = robot_dfs[source_robot_id]
        graph_to_gt = estimate_position_alignment(
            point_positions(reference_df, "graph"),
            point_positions(reference_df, "gt"),
        )
        gt_to_graph = invert_alignment(graph_to_gt)

        fig, axis = plt.subplots(figsize=(8.2, 8.2))
        color_cycle = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
        for color_idx, robot_id in enumerate(sorted(robot_dfs)):
            df = robot_dfs[robot_id]
            if df.empty:
                continue
            color = color_cycle[color_idx % len(color_cycle)] if color_cycle else None
            graph_positions = point_positions(df, "graph")
            gt_positions = gt_to_graph.apply(
                point_positions(df, "gt"),
                Rotation.identity(len(df)),
            )[0]
            axis.plot(
                graph_positions[:, 0],
                graph_positions[:, 1],
                linewidth=1.8,
                color=color,
                label=f"r{robot_id} graph",
            )
            axis.plot(
                gt_positions[:, 0],
                gt_positions[:, 1],
                linewidth=1.2,
                linestyle="--",
                color=color,
                alpha=0.7,
                label=f"r{robot_id} GT",
            )
            axis.scatter(
                graph_positions[0, 0],
                graph_positions[0, 1],
                marker="o",
                s=28,
                color=color,
                edgecolors="black",
                linewidths=0.4,
                zorder=4,
            )
            axis.scatter(
                graph_positions[-1, 0],
                graph_positions[-1, 1],
                marker="s",
                s=28,
                color=color,
                edgecolors="black",
                linewidths=0.4,
                zorder=4,
            )
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel(f"{source} frame x [m]")
        axis.set_ylabel(f"{source} frame y [m]")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize="small", ncol=2)
        axis.set_title(f"{source}: final robot-centric global graph")
        fig.tight_layout()
        fig.savefig(subdirs["trajectory"] / f"trajectory_{source}_global.png", dpi=160)
        plt.close(fig)

    fig, axis = plt.subplots(figsize=(12, 6))
    for (source, robot_id), group in metrics.groupby(["source", "robot_id"]):
        group = group.sort_values("snapshot_seq")
        axis.plot(
            group["time_rel_sec"] / 60.0,
            group["ate_m_rmse"],
            label=f"{source} r{robot_id}",
            linewidth=1.6,
        )
    axis.set_xlabel("Snapshot time from first graph [min]")
    axis.set_ylabel("ATE RMSE [m]")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize="small")
    axis.set_title("ATE RMSE overview")
    fig.tight_layout()
    fig.savefig(subdirs["ate_loops"] / "ate_rmse_overview.png", dpi=160)
    plt.close(fig)

    rendezvous_rows: list[dict[str, object]] = []
    for (source, robot_id), group in metrics_by_source_robot.items():
        inter_indices = group.index[group["new_inter_loop_closures"] > 0].to_list()
        if not inter_indices:
            inter_indices = group.index[group["inter_loop_closures"] > 0].to_list()
        if not inter_indices:
            continue
        event_idx = int(inter_indices[0])
        before_idx = max(0, event_idx - 1)
        after_idx = min(len(group) - 1, event_idx + 1)
        final_idx = len(group) - 1
        for phase, idx in [
            ("Before", before_idx),
            ("Rendezvous", event_idx),
            ("After 1", after_idx),
            ("Final", final_idx),
        ]:
            rendezvous_rows.append(
                {
                    "source": source,
                    "robot_id": robot_id,
                    "phase": phase,
                    "ate_m_rmse": float(group.loc[idx, "ate_m_rmse"]),
                }
            )
    if rendezvous_rows:
        rendezvous_df = pd.DataFrame(rendezvous_rows)
        phase_order = ["Before", "Rendezvous", "After 1", "Final"]
        for source, source_group in rendezvous_df.groupby("source"):
            robots = sorted(source_group["robot_id"].unique())
            x_base = np.arange(len(phase_order), dtype=float)
            width = 0.8 / max(len(robots), 1)
            fig, axis = plt.subplots(figsize=(9, 5.5))
            for ridx, robot_id in enumerate(robots):
                robot_group = source_group[source_group["robot_id"] == robot_id]
                values = [
                    float(
                        robot_group[robot_group["phase"] == phase]["ate_m_rmse"].iloc[0]
                    )
                    if not robot_group[robot_group["phase"] == phase].empty
                    else math.nan
                    for phase in phase_order
                ]
                offsets = x_base - 0.4 + width / 2.0 + ridx * width
                axis.bar(offsets, values, width=width, label=f"r{int(robot_id)}")
            axis.set_xticks(x_base)
            axis.set_xticklabels(phase_order)
            axis.set_ylabel("ATE RMSE [m]")
            axis.set_xlabel("Rendezvous phase")
            axis.grid(True, axis="y", alpha=0.25)
            axis.legend(loc="best")
            axis.set_title(f"{source}: ATE before/after rendezvous/final")
            fig.tight_layout()
            fig.savefig(
                subdirs["rendezvous"] / f"ate_rendezvous_{source}.png",
                dpi=160,
            )
            plt.close(fig)


def write_report(
    *,
    output_dir: Path,
    run_dir: Path,
    args: argparse.Namespace,
    metrics: pd.DataFrame,
    summary: pd.DataFrame,
    events: pd.DataFrame,
    corr: pd.DataFrame,
    loop_diagnostics: pd.DataFrame,
    communication_summary: dict[str, object] | None,
    communication_topics: pd.DataFrame,
    communication_robot_topics: pd.DataFrame,
) -> None:
    report_path = output_dir / "report.md"
    lines: list[str] = []
    source_labels = sorted({str(source) for source in metrics["source"].dropna().unique()})
    uses_mrg = any(label.startswith("mrg_") for label in source_labels)
    uses_swarm = any(label.startswith("swarm_") for label in source_labels)

    lines.append(f"# Pose Graph Run Analysis: {run_dir.name}")
    lines.append("")
    lines.append("## Configuration")
    lines.append("")
    lines.append(f"- Run: `{run_dir}`")
    lines.append(f"- Alignment: `{args.alignment}`")
    lines.append(
        "- Alignment guide: use `umeyama` for global post-optimization ATE, "
        "`first` for drift from the real initial pose, and `none` for TF/frame debugging."
    )
    lines.append(
        f"- Keyframe/GT fallback association: `{args.association}` "
        "(used only when per-keyframe ROS timestamps are unavailable)"
    )
    lines.append(
        f"- RPE keyframe deltas: `{', '.join(str(v) for v in args.rpe_keyframe_deltas)}`"
    )
    lines.append(
        "- RPE distance deltas: "
        f"`{', '.join(delta_label(v) + ' m' for v in args.rpe_distance_deltas)}`"
    )
    lines.append(
        "- Valid inter-robot loop threshold: "
        f"`{args.loop_valid_trans_threshold_m:.3f} m`, "
        f"`{args.loop_valid_rot_threshold_deg:.3f} deg`"
    )
    loop_direction = getattr(args, "loop_measurement_direction", "auto")
    lines.append(f"- Loop measurement direction: `{loop_direction}`")
    lines.append(
        "- Inter-robot opportunity radius for recall: "
        f"`{args.inter_loop_opportunity_radius_m:.3f} m`"
    )
    if source_labels:
        lines.append(f"- Sources: `{', '.join(source_labels)}`")
    lines.append("")
    lines.append("## Association Notes")
    lines.append("")
    if uses_swarm:
        lines.append(
            "New Swarm node CSVs include ROS timestamps captured from "
            "`cslam/keyframe_odom`, so graph poses are matched to ground truth "
            "directly in time. If only some nodes have a valid stamp, only the "
            "missing nodes fall back to the selected manual association mode."
        )
        lines.append("")
    if uses_mrg:
        lines.append(
            "MRG node CSVs include ROS timestamps per keyframe, so graph poses "
            "are matched to ground truth directly in time and the manual "
            "association mode is ignored."
        )
        lines.append("")
    lines.append("## How To Read The Metrics")
    lines.append("")
    lines.append(
        "`graph_*` columns are global counts/metrics for the whole pose graph. "
        "Columns without `graph_` are incident to the analyzed robot. An inter-robot "
        "loop closure connecting r0 and r2 is therefore counted once in "
        "`graph_inter_loop_closures`, but appears in both r0 and r2 incident rows."
    )
    lines.append("")
    lines.append(
        "`valid_inter_robot_loop_closures` validates accepted inter-robot edges "
        "against ground truth relative pose. Precision is valid/evaluated accepted "
        "inter-robot closures. Recall is an estimate: valid accepted closures that "
        "also fall within the GT opportunity radius divided by all inter-robot "
        "keyframe pairs within that radius."
    )
    lines.append("")
    lines.append(
        "`global_map_consistency_error_m` is the graph-wide translational RMSE "
        "between optimized inter-robot relative poses and their loop-closure "
        "measurements. It is a pose-graph consistency proxy, not a dense map error."
    )
    lines.append("")
    lines.append("## Final Trajectory Metrics")
    lines.append("")
    if summary.empty:
        lines.append("No metrics were produced.")
    else:
        display_columns = [
            "source",
            "robot_id",
            "final_snapshot_seq",
            "association_used",
            "node_count_robot",
            "ate_m_rmse",
            "ate_m_max",
            "ate_m_final",
            "drift_percent",
            "scale_ratio_graph_to_gt",
        ]
        existing = [col for col in display_columns if col in summary.columns]
        lines.append(dataframe_to_markdown(summary[existing], floatfmt=".4f"))
    lines.append("")
    lines.append("## Final RPE By Keyframe Delta")
    lines.append("")
    if summary.empty:
        lines.append("No RPE metrics were produced.")
    else:
        rpe_columns = ["source", "robot_id"]
        for delta in args.rpe_keyframe_deltas:
            prefix = f"rpe_kf_{delta}"
            rpe_columns.extend(
                [
                    f"{prefix}_trans_rmse_m",
                    f"{prefix}_rot_rmse_deg",
                    f"{prefix}_pairs",
                ]
            )
        existing = [col for col in rpe_columns if col in summary.columns]
        lines.append(dataframe_to_markdown(summary[existing], floatfmt=".4f"))
    lines.append("")
    lines.append("## Final RPE By Distance Delta")
    lines.append("")
    if summary.empty:
        lines.append("No distance RPE metrics were produced.")
    else:
        rpe_columns = ["source", "robot_id"]
        for delta_m in args.rpe_distance_deltas:
            prefix = f"rpe_dist_{delta_label(delta_m)}m"
            rpe_columns.extend(
                [
                    f"{prefix}_trans_rmse_m",
                    f"{prefix}_rot_rmse_deg",
                    f"{prefix}_pairs",
                ]
            )
        existing = [col for col in rpe_columns if col in summary.columns]
        lines.append(dataframe_to_markdown(summary[existing], floatfmt=".4f"))
    lines.append("")
    lines.append("## Inter-Robot Loop Closure Quality")
    lines.append("")
    if summary.empty:
        lines.append("No loop-closure quality metrics were produced.")
    else:
        graph_columns = [
            "source",
            "final_snapshot_seq",
            "graph_inter_loop_closures",
            "graph_valid_inter_robot_loop_closures",
            "graph_inter_robot_loop_closure_precision",
            "graph_real_inter_robot_loop_opportunities",
            "graph_inter_robot_loop_closure_recall",
            "global_map_consistency_error_m",
            "graph_map_consistency_rot_rmse_deg",
        ]
        graph_view = (
            summary.sort_values(["source", "final_snapshot_seq"])
            .drop_duplicates(subset=["source"], keep="last")
        )
        existing = [col for col in graph_columns if col in graph_view.columns]
        lines.append("Graph-wide final values:")
        lines.append("")
        lines.append(dataframe_to_markdown(graph_view[existing], floatfmt=".4f"))
        lines.append("")
        incident_columns = [
            "source",
            "robot_id",
            "inter_loop_closures",
            "valid_inter_robot_loop_closures",
            "inter_robot_loop_closure_precision",
            "real_inter_robot_loop_opportunities",
            "inter_robot_loop_closure_recall",
            "map_consistency_error_m",
        ]
        existing = [col for col in incident_columns if col in summary.columns]
        lines.append("Incident-to-robot final values:")
        lines.append("")
        lines.append(dataframe_to_markdown(summary[existing], floatfmt=".4f"))
    lines.append("")
    lines.append("## Inter-Robot Loop Closure Diagnostic")
    lines.append("")
    if loop_diagnostics.empty:
        lines.append("No final inter-robot loop-closure diagnostics were produced.")
    else:
        diag_summary = (
            loop_diagnostics.groupby(["source", "selected_measurement_direction"])
            .agg(
                edges=("current_valid", "count"),
                current_valid=("current_valid", "sum"),
                inverted_valid=("inverted_valid", "sum"),
                planar_valid=("planar_valid", "sum"),
                planar_inverted_valid=("planar_inverted_valid", "sum"),
                current_rot_mean_deg=("current_rot_error_deg", "mean"),
                inverted_rot_mean_deg=("inverted_rot_error_deg", "mean"),
                planar_yaw_mean_deg=("planar_yaw_error_deg", "mean"),
                planar_inverted_yaw_mean_deg=("planar_inverted_yaw_error_deg", "mean"),
            )
            .reset_index()
        )
        lines.append(dataframe_to_markdown(diag_summary, floatfmt=".4f"))
        lines.append("")
        lines.append(
            "`current_valid` uses the edge measurement as stored. "
            "`inverted_valid` tests whether the same measurement looks correct "
            "when inverted. `planar_valid` compares only XY translation and yaw. "
            "A much larger inverted or planar count points to a direction/frame "
            "convention issue rather than simply bad loop closures."
        )
    lines.append("")
    lines.append("## ATE Reduction After Rendezvous")
    lines.append("")
    if summary.empty:
        lines.append("No rendezvous metrics were produced.")
    else:
        rendezvous_columns = [
            "source",
            "robot_id",
            "first_inter_robot_loop_snapshot_seq",
            "ate_rmse_before_first_inter_loop_m",
            "ate_rmse_at_first_inter_loop_m",
            "ate_m_rmse",
            "ate_reduction_after_rendezvous_m",
            "ate_reduction_after_rendezvous_percent",
        ]
        existing = [col for col in rendezvous_columns if col in summary.columns]
        lines.append(dataframe_to_markdown(summary[existing], floatfmt=".4f"))
    lines.append("")
    lines.append("## Loop Closure Impact On Graph Correction")
    lines.append("")
    if events.empty:
        lines.append("No new loop-closure events were detected in analyzed snapshots.")
    else:
        event_summary = (
            events.groupby(["source", "robot_id"])
            .agg(
                events=("snapshot_seq", "count"),
                new_intra=("new_intra_loop_closures", "sum"),
                new_inter=("new_inter_loop_closures", "sum"),
                new_valid_inter=("new_valid_inter_robot_loop_closures", "sum"),
                mean_immediate_delta_m=("ate_delta_current_minus_before_m", "mean"),
                mean_after_1_delta_m=("ate_delta_after_1_minus_current_m", "mean"),
                mean_graph_correction_rmse_m=("graph_correction_trans_rmse_m", "mean"),
                mean_graph_correction_rot_deg=("graph_correction_rot_rmse_deg", "mean"),
                mean_map_consistency_delta_m=(
                    "map_consistency_delta_current_minus_before_m",
                    "mean",
                ),
            )
            .reset_index()
        )
        lines.append(dataframe_to_markdown(event_summary, floatfmt=".4f"))
        lines.append("")
        lines.append(
            "Negative ATE or map-consistency deltas indicate a reduction after a "
            "loop-closure update. `graph_correction_*` measures how much common "
            "keyframes moved between consecutive snapshots."
        )
    lines.append("")
    lines.append("## Strongest Correlations")
    lines.append("")
    if corr.empty:
        lines.append("No correlations were computed.")
    else:
        corr_view = corr.copy()
        corr_view["abs_spearman"] = corr_view[
            "spearman_corr_with_ate_rmse"
        ].abs()
        corr_view = corr_view.sort_values("abs_spearman", ascending=False).head(12)
        keep = [
            "source",
            "robot_id",
            "variable",
            "spearman_corr_with_ate_rmse",
            "pearson_corr_with_ate_rmse",
            "linear_slope_ate_rmse_per_unit",
        ]
        lines.append(dataframe_to_markdown(corr_view[keep], floatfmt=".4f"))
    lines.append("")
    if communication_summary is not None:
        lines.append("## Communication Metrics")
        lines.append("")
        lines.append(
            "These values are estimated logical inter-robot communication from "
            "serialized ROS message payloads. They are not physical DDS or "
            "network bandwidth measurements."
        )
        lines.append("")
        topic_total_mb = communication_summary.get(
            "topic_total_MB",
            communication_summary.get("total_MB", 0.0),
        )
        service_total_mb = communication_summary.get("service_total_MB", 0.0)
        combined_total_mb = communication_summary.get(
            "combined_total_MB",
            float(topic_total_mb) + float(service_total_mb),
        )
        service_events = communication_summary.get("service_event_count", 0)
        overview = pd.DataFrame(
            [
                {
                    "slam_type": communication_summary.get("slam_type", ""),
                    "duration_sec": communication_summary.get("duration_sec", 0.0),
                    "total_messages": communication_summary.get(
                        "total_messages", 0
                    ),
                    "topic_total_MB": topic_total_mb,
                    "service_total_MB": service_total_mb,
                    "combined_total_MB": combined_total_mb,
                    "service_events": service_events,
                    "average_bandwidth_MBps": communication_summary.get(
                        "average_bandwidth_MBps", 0.0
                    ),
                    "peak_bandwidth_MBps": communication_summary.get(
                        "peak_bandwidth_MBps", 0.0
                    ),
                }
            ]
        )
        lines.append(dataframe_to_markdown(overview, floatfmt=".6f"))
        lines.append("")
        if communication_topics.empty:
            lines.append("No communication topic rows were recorded.")
        else:
            top_topics = communication_topics.sort_values(
                "total_bytes",
                ascending=False,
            ).head(8)
            display_columns = [
                "topic",
                "message_type",
                "message_count",
                "total_MB",
                "average_bandwidth_Bps",
                "peak_bandwidth_Bps",
            ]
            existing = [
                column for column in display_columns if column in top_topics.columns
            ]
            lines.append("Biggest measured communication topics:")
            lines.append("")
            lines.append(dataframe_to_markdown(top_topics[existing], floatfmt=".4f"))

        per_robot_tx = communication_summary.get("per_robot_tx_estimate", {})
        per_robot_rx = communication_summary.get("per_robot_rx_estimate", {})
        per_robot_unknown = communication_summary.get(
            "per_robot_unknown_bytes",
            {},
        )
        if isinstance(per_robot_tx, dict) and per_robot_tx:
            rx_keys = per_robot_rx.keys() if isinstance(per_robot_rx, dict) else []
            unknown_keys = (
                per_robot_unknown.keys()
                if isinstance(per_robot_unknown, dict)
                else []
            )
            robots = sorted(
                {
                    *[str(key) for key in per_robot_tx.keys()],
                    *[str(key) for key in rx_keys],
                    *[str(key) for key in unknown_keys],
                }
            )
            robot_rows = []
            for robot in robots:
                tx_bytes = float(per_robot_tx.get(robot, 0) or 0)
                rx_bytes = (
                    float(per_robot_rx.get(robot, 0) or 0)
                    if isinstance(per_robot_rx, dict)
                    else 0.0
                )
                unknown_bytes = (
                    float(per_robot_unknown.get(robot, 0) or 0)
                    if isinstance(per_robot_unknown, dict)
                    else 0.0
                )
                robot_rows.append(
                    {
                        "robot": robot,
                        "tx_estimate_MB": tx_bytes / 1_000_000.0,
                        "rx_estimate_MB": rx_bytes / 1_000_000.0,
                        "unknown_peer_MB": unknown_bytes / 1_000_000.0,
                    }
                )
            lines.append("")
            lines.append("Per-robot communication attribution:")
            lines.append("")
            lines.append(dataframe_to_markdown(pd.DataFrame(robot_rows), floatfmt=".4f"))
            unattributed_bytes = float(
                communication_summary.get("unattributed_bytes", 0) or 0
            )
            combined_bytes = float(
                communication_summary.get("combined_total_bytes", 0) or 0
            )
            if combined_bytes > 0.0 and unattributed_bytes / combined_bytes > 0.05:
                lines.append("")
                lines.append(
                    f"Warning: {unattributed_bytes / 1_000_000.0:.4f} MB "
                    "of communication could not be attributed to a source robot."
                )

        if not communication_robot_topics.empty:
            top_robot_topics = communication_robot_topics.sort_values(
                "total_bytes",
                ascending=False,
            ).head(12)
            display_columns = [
                "source_robot",
                "peer_robot",
                "topic",
                "traffic_class",
                "message_count",
                "total_MB",
                "average_bandwidth_Bps",
                "peak_bandwidth_Bps",
            ]
            existing = [
                column
                for column in display_columns
                if column in top_robot_topics.columns
            ]
            lines.append("")
            lines.append("Top per-robot communication topics:")
            lines.append("")
            lines.append(
                dataframe_to_markdown(top_robot_topics[existing], floatfmt=".4f")
            )

        attribution_notes = communication_summary.get("attribution_notes", [])
        if attribution_notes:
            lines.append("")
            lines.append("Attribution notes:")
            lines.append("")
            for item in attribution_notes:
                lines.append(f"- {item}")
        notes_and_warnings = [
            *communication_summary.get("notes", []),
            *communication_summary.get("warnings", []),
        ]
        if notes_and_warnings:
            lines.append("")
            lines.append("Recorder notes and warnings:")
            lines.append("")
            for item in notes_and_warnings:
                lines.append(f"- {item}")
        lines.append("")
    lines.append("## Outputs")
    lines.append("")
    lines.append(
        "- `ate_timeseries.csv`: ATE/RPE, drift, loop quality, graph correction, "
        "and graph consistency per snapshot."
    )
    lines.append(
        "- `summary_metrics.csv`: final, best/worst, loop-quality, and rendezvous metrics."
    )
    lines.append(
        "- `loop_closure_events.csv`: ATE, map-consistency, and pose-correction "
        "deltas around new closures."
    )
    lines.append("- `loop_ate_correlations.csv`: correlation/slope diagnostics.")
    lines.append(
        "- `inter_robot_loop_diagnostics.csv`: final inter-robot loop closures "
        "checked as stored, inverted, and planar XY+yaw."
    )
    lines.append("- `alignment_transforms.csv`: homogeneous graph-to-GT transform per snapshot.")
    lines.append("- `point_errors/`: final per-keyframe errors and aligned positions.")
    lines.append(
        "- `plots/`: PNG plots grouped by type (`ate_loops/`, `loop_quality/`, "
        "`rendezvous/`, `graph_correction/`, `rpe/`, `trajectory/`, "
        "`final_ate/`, `communication/`). Each folder includes a `README.md` "
        "explaining the plots."
    )
    if communication_summary is not None:
        lines.append(
            "- `communication/`: estimated logical inter-robot communication "
            "summary, topic/service totals, and bandwidth time series."
        )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(args: argparse.Namespace) -> Path:
    run_dir = args.run if args.run is not None else latest_run(args.results_root)
    run_dir = run_dir.resolve()
    if not run_dir.exists():
        raise FileNotFoundError(f"Run directory does not exist: {run_dir}")

    output_dir = args.output.resolve() if args.output else run_dir / "analysis_pose_graph"
    output_dir.mkdir(parents=True, exist_ok=True)

    robot_filter = selected_robot_ids(args.robots)
    ground_truth = load_ground_truth(run_dir)
    if robot_filter is not None:
        ground_truth = {
            robot_id: gt for robot_id, gt in ground_truth.items() if robot_id in robot_filter
        }
    if not ground_truth:
        raise RuntimeError("No usable ground-truth trajectories found.")

    sources = discover_sources(run_dir, args.sources)
    if not sources:
        raise RuntimeError("No pose-graph sources with snapshots.csv found.")

    max_keyframes = compute_max_keyframes(run_dir, sources)
    metric_rows: list[dict[str, object]] = []
    transform_rows: list[dict[str, object]] = []
    loop_diagnostic_rows: list[dict[str, object]] = []
    point_errors: dict[tuple[str, int], pd.DataFrame] = {}

    for source_dir in sources:
        snapshots = pd.read_csv(source_dir / "snapshots.csv")
        if snapshots.empty:
            continue
        final_snapshot_seq_for_source = int(snapshots["snapshot_seq"].max())
        final_seq_by_robot: dict[int, int] = {}
        for _, snapshot in snapshots.iterrows():
            nodes_path = run_dir / str(snapshot["nodes_file"])
            if not nodes_path.exists():
                continue
            nodes = read_nodes(nodes_path)
            if nodes.empty:
                continue
            for robot_id in sorted(set(nodes["robot_id"].astype(int))):
                if robot_id in ground_truth:
                    final_seq_by_robot[robot_id] = int(snapshot["snapshot_seq"])

        loop_measurement_direction = args.loop_measurement_direction
        if loop_measurement_direction == "auto":
            loop_measurement_direction = "as-stored"
            final_snapshot = snapshots[
                snapshots["snapshot_seq"].astype(int) == final_snapshot_seq_for_source
            ].iloc[-1]
            final_nodes_path = run_dir / str(final_snapshot["nodes_file"])
            final_edges_path = run_dir / str(final_snapshot["edges_file"])
            if final_nodes_path.exists() and final_edges_path.exists():
                final_nodes = read_nodes(final_nodes_path)
                if not final_nodes.empty:
                    final_edges = read_edges(final_edges_path, nodes=final_nodes)
                    final_gt_pose_lookup = build_ground_truth_pose_lookup(
                        final_nodes,
                        ground_truth,
                        max_keyframes,
                        source_dir.name,
                        args.association,
                    )
                    loop_measurement_direction = infer_loop_measurement_direction(
                        final_edges,
                        final_gt_pose_lookup,
                        trans_threshold_m=args.loop_valid_trans_threshold_m,
                        rot_threshold_deg=args.loop_valid_rot_threshold_deg,
                    )

        previous_nodes: pd.DataFrame | None = None
        for _, snapshot in snapshots.iterrows():
            nodes_path = run_dir / str(snapshot["nodes_file"])
            edges_path = run_dir / str(snapshot["edges_file"])
            if not nodes_path.exists() or not edges_path.exists():
                continue
            nodes = read_nodes(nodes_path)
            if nodes.empty:
                continue
            edges = read_edges(edges_path, nodes=nodes)
            graph_counts = edge_counts(edges)
            graph_consistency, incident_consistency_by_robot = map_consistency_metrics_all(
                edges,
                nodes,
                loop_measurement_direction,
            )
            gt_pose_lookup = build_ground_truth_pose_lookup(
                nodes,
                ground_truth,
                max_keyframes,
                source_dir.name,
                args.association,
            )
            graph_opportunities, incident_opportunities = inter_robot_opportunity_counts(
                nodes,
                gt_pose_lookup,
                args.inter_loop_opportunity_radius_m,
            )
            graph_loop_quality, incident_loop_quality_by_robot = (
                inter_robot_loop_quality_metrics_all(
                    edges,
                    nodes,
                    gt_pose_lookup,
                    trans_threshold_m=args.loop_valid_trans_threshold_m,
                    rot_threshold_deg=args.loop_valid_rot_threshold_deg,
                    measurement_direction=loop_measurement_direction,
                    opportunity_radius_m=args.inter_loop_opportunity_radius_m,
                    graph_opportunities=graph_opportunities,
                    incident_opportunities=incident_opportunities,
                )
            )
            if int(snapshot["snapshot_seq"]) == final_snapshot_seq_for_source:
                loop_diagnostic_rows.extend(
                    inter_robot_loop_diagnostics(
                        source=source_dir.name,
                        snapshot=snapshot,
                        edges=edges,
                        gt_lookup=gt_pose_lookup,
                        selected_measurement_direction=loop_measurement_direction,
                        trans_threshold_m=args.loop_valid_trans_threshold_m,
                        rot_threshold_deg=args.loop_valid_rot_threshold_deg,
                        opportunity_radius_m=args.inter_loop_opportunity_radius_m,
                    )
                )
            robots_in_snapshot = sorted(set(nodes["robot_id"].astype(int)))
            for robot_id in robots_in_snapshot:
                if robot_id not in ground_truth:
                    continue
                incident_consistency = incident_consistency_by_robot.get(
                    robot_id,
                    {
                        **trans_rot_empty_stats("map_consistency"),
                        "map_consistency_error_m": math.nan,
                    },
                )
                incident_loop_quality = incident_loop_quality_by_robot.get(
                    robot_id,
                    empty_loop_quality_metrics(
                        "",
                        incident_opportunities.get(robot_id, 0),
                    ),
                )
                correction = graph_correction_metrics(
                    previous_nodes,
                    nodes,
                    robot_id,
                )
                counts = {
                    **{f"graph_{key}": value for key, value in graph_counts.items()},
                    **graph_consistency,
                    **graph_loop_quality,
                    **edge_counts(edges, robot_id=robot_id),
                    **incident_consistency,
                    **incident_loop_quality,
                    **correction,
                }
                max_keyframe_id = max_keyframes.get((source_dir.name, robot_id), 0)
                snapshot_for_robot = snapshot.copy()
                snapshot_for_robot["_final_seq_for_source_robot"] = final_seq_by_robot.get(
                    robot_id,
                    -1,
                )
                try:
                    row, transform_row, point_df = analyze_snapshot_robot(
                        run_dir=run_dir,
                        source=source_dir.name,
                        snapshot=snapshot_for_robot,
                        robot_id=robot_id,
                        nodes=nodes,
                        edge_count_values=counts,
                        gt=ground_truth[robot_id],
                        max_keyframe_id=max_keyframe_id,
                        alignment_mode=args.alignment,
                        association=args.association,
                        rpe_keyframe_deltas=args.rpe_keyframe_deltas,
                        rpe_distance_deltas_m=args.rpe_distance_deltas,
                    )
                except Exception as exc:
                    print(
                        f"Warning: skipped {source_dir.name} seq={snapshot['snapshot_seq']} "
                        f"robot={robot_id}: {exc}",
                        file=sys.stderr,
                    )
                    continue
                metric_rows.append(row)
                transform_rows.append(transform_row)
                if point_df is not None:
                    point_errors[(source_dir.name, robot_id)] = point_df
            previous_nodes = nodes

    metrics = pd.DataFrame(metric_rows)
    if metrics.empty:
        raise RuntimeError("No snapshot metrics could be computed.")
    metrics = add_time_and_deltas(metrics)
    transforms = pd.DataFrame(transform_rows)
    loop_diagnostics = pd.DataFrame(loop_diagnostic_rows)
    corr = correlations(metrics)
    events = loop_events(metrics, args.event_window)
    summary = final_summary(metrics)
    communication_summary = load_communication_summary(run_dir)
    communication_topics = load_communication_topics(run_dir)
    communication_robot_topics = load_communication_robot_topics(run_dir)

    metrics.to_csv(output_dir / "ate_timeseries.csv", index=False)
    transforms.to_csv(output_dir / "alignment_transforms.csv", index=False)
    corr.to_csv(output_dir / "loop_ate_correlations.csv", index=False)
    events.to_csv(output_dir / "loop_closure_events.csv", index=False)
    loop_diagnostics.to_csv(output_dir / "inter_robot_loop_diagnostics.csv", index=False)
    summary.to_csv(output_dir / "summary_metrics.csv", index=False)
    write_point_errors(point_errors, output_dir)
    if not args.no_plots:
        make_plots(metrics, point_errors, output_dir)
        make_communication_plots(run_dir, output_dir)
    write_report(
        output_dir=output_dir,
        run_dir=run_dir,
        args=args,
        metrics=metrics,
        summary=summary,
        events=events,
        corr=corr,
        loop_diagnostics=loop_diagnostics,
        communication_summary=communication_summary,
        communication_topics=communication_topics,
        communication_robot_topics=communication_robot_topics,
    )
    return output_dir


def main() -> int:
    args = parse_args()
    try:
        output_dir = analyze(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    print(f"Analysis written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
