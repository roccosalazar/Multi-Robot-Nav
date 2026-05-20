#!/usr/bin/env python3

from __future__ import annotations

import copy
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time
from typing import Dict, TextIO

from cslam_common_interfaces.msg import KeyframeOdom, PoseGraph
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from slam_evaluation.result_utils import (
    FileEventLogger,
    make_run_directory,
    now_wall_time,
    parse_csv,
    sanitize_filename,
)

try:
    from mrg_slam_msgs.msg import SlamStatus
    from mrg_slam_msgs.srv import PublishGraph
except ImportError:
    # Let Swarm-only installs still import this node.
    SlamStatus = None
    PublishGraph = None


@dataclass
class SourceFiles:
    directory: Path
    snapshot_file: TextIO
    snapshot_writer: csv.writer
    sequence: int = 0
    last_signature: str = ''


class PoseGraphRecorder(Node):
    """Record Swarm PoseGraph topics and MRG graph service snapshots to CSV."""

    SNAPSHOT_HEADER = [
        'snapshot_seq',
        'wall_time_unix_sec',
        'ros_time_sec',
        'ros_time_nsec',
        'source',
        'source_type',
        'reason',
        'robot_id',
        'origin_robot_id',
        'robot_name',
        'topic_or_service',
        'node_count',
        'edge_count',
        'nodes_file',
        'edges_file',
        'signature',
    ]

    SWARM_NODE_HEADER = [
        'snapshot_seq',
        'source',
        'robot_id',
        'keyframe_id',
        'stamp_sec',
        'stamp_nsec',
        'position_x',
        'position_y',
        'position_z',
        'orientation_x',
        'orientation_y',
        'orientation_z',
        'orientation_w',
    ]

    SWARM_EDGE_HEADER = [
        'snapshot_seq',
        'source',
        'edge_type',
        'from_robot_id',
        'from_keyframe_id',
        'to_robot_id',
        'to_keyframe_id',
        'measurement_x',
        'measurement_y',
        'measurement_z',
        'measurement_qx',
        'measurement_qy',
        'measurement_qz',
        'measurement_qw',
        'noise_0',
        'noise_1',
        'noise_2',
        'noise_3',
        'noise_4',
        'noise_5',
    ]

    MRG_NODE_HEADER = [
        'snapshot_seq',
        'source',
        'graph_robot_name',
        'keyframe_robot_name',
        'uuid',
        'slam_uuid',
        'odom_counter',
        'stamp_sec',
        'stamp_nsec',
        'accum_distance',
        'first_keyframe',
        'static_keyframe',
        'position_x',
        'position_y',
        'position_z',
        'orientation_x',
        'orientation_y',
        'orientation_z',
        'orientation_w',
    ]

    MRG_EDGE_HEADER = [
        'snapshot_seq',
        'source',
        'uuid',
        'from_uuid',
        'to_uuid',
        'edge_type',
        'edge_type_id',
        'relative_x',
        'relative_y',
        'relative_z',
        'relative_qx',
        'relative_qy',
        'relative_qz',
        'relative_qw',
    ]

    def __init__(self) -> None:
        super().__init__('pose_graph_recorder')

        self.declare_parameter('mode', 'auto')
        self.declare_parameter('robot_names', '')
        self.declare_parameter('results_root', '')
        self.declare_parameter('run_type', 'run')
        self.declare_parameter(
            'run_id',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter(
            'swarm_pose_graph_topics',
            '/cslam/viz/pose_graph',
        )
        self.declare_parameter('swarm_record_merged_graphs', False)
        self.declare_parameter('swarm_robot_centric_global_graphs', True)
        self.declare_parameter(
            'swarm_keyframe_odom_topic_template',
            '/{robot_name}/cslam/keyframe_odom',
        )
        self.declare_parameter('mrg_poll_period_sec', 1.0)
        self.declare_parameter('signature_precision', 9)

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        if self.mode not in ('auto', 'swarm', 'mrg'):
            raise ValueError(
                "Parameter 'mode' must be one of: auto, swarm, mrg"
            )

        self.robot_names = self._resolve_robot_names()
        self.signature_precision = int(
            self.get_parameter('signature_precision').value
        )
        self.run_dir = make_run_directory(
            str(self.get_parameter('results_root').value),
            str(self.get_parameter('run_type').value),
            str(self.get_parameter('run_id').value),
        )
        self.output_dir = self.run_dir / 'pose_graph'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file_logger = FileEventLogger(
            self,
            self.output_dir / 'pose_graph_recorder.log',
        )

        self._sources: Dict[str, SourceFiles] = {}
        self._subscriptions = []
        self._mrg_clients = {}
        self._mrg_service_names = {}
        self._mrg_pending = {}
        self._mrg_statuses = {}
        self._mrg_last_service_warn: Dict[str, float] = {}
        self._swarm_pose_graph_cache: Dict[int, PoseGraph] = {}
        self._swarm_keyframe_stamps: Dict[
            tuple[int, int], tuple[int, int]
        ] = {}
        self._swarm_robot_centric_global_graphs = self._as_bool(
            self.get_parameter('swarm_robot_centric_global_graphs').value
        )

        if self.mode in ('auto', 'swarm'):
            self._start_swarm_recording()

        if self.mode in ('auto', 'mrg'):
            self._start_mrg_recording()

        self.file_logger.info(
            f"Pose graph recorder ready mode='{self.mode}' "
            f"robots={self.robot_names} "
            f"output='{self.output_dir}'"
        )

    def _resolve_robot_names(self) -> list[str]:
        robot_names = parse_csv(self.get_parameter('robot_names').value)
        if robot_names:
            return robot_names

        namespace = self.get_namespace().strip('/')
        if namespace:
            return [namespace]

        return ['r0']

    def _as_bool(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('true', '1', 'yes', 'on')

    def _start_swarm_recording(self) -> None:
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE

        topics = parse_csv(self.get_parameter('swarm_pose_graph_topics').value)
        if not topics:
            topics = ['/cslam/viz/pose_graph']

        if self._as_bool(
            self.get_parameter('swarm_record_merged_graphs').value
        ):
            topics.extend(
                f'/{robot_name}/cslam/viz/merged_pose_graph'
                for robot_name in self.robot_names
            )

        unique_topics = []
        for topic in topics:
            normalized_topic = topic if topic.startswith('/') else f'/{topic}'
            if normalized_topic not in unique_topics:
                unique_topics.append(normalized_topic)

        for topic in unique_topics:
            source = self._source_name_from_swarm_topic(topic)
            self._subscriptions.append(
                self.create_subscription(
                    PoseGraph,
                    topic,
                    lambda msg, src=source, top=topic: (
                        self._swarm_pose_graph_callback(msg, src, top)
                    ),
                    qos,
                )
            )
            self.file_logger.info(
                f"Recording Swarm pose graph source='{source}' "
                f"topic='{topic}'"
            )

        self._start_swarm_keyframe_odom_recording(qos)

    def _start_swarm_keyframe_odom_recording(self, qos: QoSProfile) -> None:
        topic_template = str(
            self.get_parameter('swarm_keyframe_odom_topic_template').value
        ).strip()
        if not topic_template:
            return

        unique_topics = set()
        for robot_name in self.robot_names:
            robot_id = self._robot_id_from_name(robot_name)
            if robot_id is None:
                self.file_logger.warn(
                    "Skipping Swarm keyframe timestamp recording for "
                    f"robot_name='{robot_name}' because no numeric robot id "
                    "could be inferred."
                )
                continue

            try:
                topic = topic_template.format(
                    robot_name=robot_name,
                    robot_id=robot_id,
                )
            except Exception as exc:
                self.file_logger.error(
                    "Invalid swarm_keyframe_odom_topic_template "
                    f"'{topic_template}': {exc}"
                )
                return

            normalized_topic = topic if topic.startswith('/') else f'/{topic}'
            if normalized_topic in unique_topics:
                continue
            unique_topics.add(normalized_topic)

            self._subscriptions.append(
                self.create_subscription(
                    KeyframeOdom,
                    normalized_topic,
                    lambda msg, rid=robot_id, top=normalized_topic: (
                        self._swarm_keyframe_odom_callback(msg, rid, top)
                    ),
                    qos,
                )
            )
            self.file_logger.info(
                "Recording Swarm keyframe timestamps "
                f"robot_id={robot_id} topic='{normalized_topic}'"
            )

    def _swarm_keyframe_odom_callback(
        self,
        msg: KeyframeOdom,
        robot_id: int,
        topic: str,
    ) -> None:
        del topic
        self._swarm_keyframe_stamps[(int(robot_id), int(msg.id))] = (
            int(msg.odom.header.stamp.sec),
            int(msg.odom.header.stamp.nanosec),
        )

    def _source_name_from_swarm_topic(self, topic: str) -> str:
        stripped = topic.strip('/')
        parts = stripped.split('/')
        if stripped == 'cslam/viz/pose_graph':
            return 'swarm_global_pose_graph'
        if (
            len(parts) >= 4
            and parts[-3:] == ['cslam', 'viz', 'merged_pose_graph']
        ):
            return f'swarm_{sanitize_filename(parts[0])}_merged_pose_graph'
        return f'swarm_{sanitize_filename(stripped)}'

    def _start_mrg_recording(self) -> None:
        if PublishGraph is None or SlamStatus is None:
            self.file_logger.error(
                "mrg_slam_msgs is not importable; "
                "MRG graph recording is disabled."
            )
            return

        for robot_name in self.robot_names:
            service_name = f'/{robot_name}/mrg_slam/publish_graph'
            status_topic = f'/{robot_name}/mrg_slam/slam_status'
            self._mrg_clients[robot_name] = self.create_client(
                PublishGraph,
                service_name,
            )
            self._mrg_service_names[robot_name] = service_name
            self._subscriptions.append(
                self.create_subscription(
                    SlamStatus,
                    status_topic,
                    lambda msg, robot=robot_name: (
                        self._mrg_status_callback(msg, robot)
                    ),
                    50,
                )
            )
            self.file_logger.info(
                f"Recording MRG graph robot='{robot_name}' "
                f"service='{service_name}' status_topic='{status_topic}'"
            )

        period = max(
            0.1,
            float(self.get_parameter('mrg_poll_period_sec').value),
        )
        self.create_timer(period, self._mrg_poll_timer_callback)

    def _swarm_pose_graph_callback(
        self,
        msg: PoseGraph,
        source: str,
        topic: str,
    ) -> None:
        if source == 'swarm_global_pose_graph':
            self._save_swarm_global_pose_graph_snapshots(msg, topic)
            return

        self._save_swarm_snapshot(
            msg,
            source,
            topic,
            'topic_update',
        )

    def _save_swarm_global_pose_graph_snapshots(
        self,
        msg: PoseGraph,
        topic: str,
    ) -> None:
        source_robot_id = int(msg.robot_id)
        self._swarm_pose_graph_cache[source_robot_id] = copy.deepcopy(msg)

        if not self._swarm_robot_centric_global_graphs:
            self._save_swarm_snapshot(
                msg,
                f'swarm_global_pose_graph_robot{source_robot_id}',
                topic,
                'topic_update',
            )
            return

        target_robot_ids = []
        for robot_name in self.robot_names:
            robot_id = self._robot_id_from_name(robot_name)
            if robot_id is not None:
                target_robot_ids.append(robot_id)

        if not target_robot_ids:
            target_robot_ids = sorted(self._swarm_pose_graph_cache.keys())

        for target_robot_id in sorted(set(target_robot_ids)):
            aggregated_msg = self._build_robot_centric_pose_graph(
                target_robot_id
            )
            if aggregated_msg is None:
                continue
            self._save_swarm_snapshot(
                aggregated_msg,
                f'swarm_global_pose_graph_robot{target_robot_id}',
                topic,
                'topic_update_robot_centric',
            )

    def _build_robot_centric_pose_graph(
        self,
        target_robot_id: int,
    ) -> PoseGraph | None:
        reference_graph = self._swarm_pose_graph_cache.get(target_robot_id)
        if reference_graph is None:
            return None

        target_origin_id = int(reference_graph.origin_robot_id)
        aggregated_msg = PoseGraph()
        aggregated_msg.robot_id = target_robot_id
        aggregated_msg.origin_robot_id = target_origin_id

        connected_robot_ids = set()

        for robot_id, cached_msg in sorted(self._swarm_pose_graph_cache.items()):
            if int(cached_msg.origin_robot_id) != target_origin_id:
                continue
            aggregated_msg.values.extend(copy.deepcopy(cached_msg.values))
            aggregated_msg.edges.extend(copy.deepcopy(cached_msg.edges))
            connected_robot_ids.add(robot_id)
            for connected_robot_id in cached_msg.connected_robots.ids:
                connected_robot_ids.add(int(connected_robot_id))

        connected_robot_ids.discard(target_robot_id)
        aggregated_msg.connected_robots.ids.extend(sorted(connected_robot_ids))
        return aggregated_msg

    def _robot_id_from_name(self, robot_name: str) -> int | None:
        stripped = robot_name.strip().lower()
        if stripped.startswith('r') and stripped[1:].isdigit():
            return int(stripped[1:])
        return None

    def _mrg_status_callback(self, msg, robot_name: str) -> None:
        previous = self._mrg_statuses.get(robot_name)
        current = (
            bool(msg.in_optimization),
            bool(msg.in_loop_closure),
            bool(msg.in_graph_exchange),
        )
        self._mrg_statuses[robot_name] = current

        was_busy = previous is not None and any(previous)
        is_idle = not any(current)
        if bool(msg.initialized) and was_busy and is_idle:
            self._request_mrg_graph(robot_name, 'status_idle_after_update')

    def _mrg_poll_timer_callback(self) -> None:
        for robot_name in self._mrg_clients:
            self._request_mrg_graph(robot_name, 'poll')

    def _request_mrg_graph(self, robot_name: str, reason: str) -> None:
        if robot_name in self._mrg_pending:
            return

        client = self._mrg_clients.get(robot_name)
        if client is None:
            return
        service_name = self._mrg_service_names.get(robot_name, '')

        if not client.service_is_ready():
            now = time.time()
            last_warn = self._mrg_last_service_warn.get(robot_name, 0.0)
            if now - last_warn > 5.0:
                self.file_logger.warn(
                    f"MRG graph service not ready robot='{robot_name}' "
                    f"service='{service_name}'"
                )
                self._mrg_last_service_warn[robot_name] = now
            return

        request = PublishGraph.Request()
        request.robot_name = 'slam_evaluation_pose_graph_recorder'
        request.processed_keyframe_uuid_strs = []
        request.processed_edge_uuid_strs = []

        future = client.call_async(request)
        self._mrg_pending[robot_name] = future
        future.add_done_callback(
            lambda done_future,
            robot=robot_name,
            service=service_name,
            why=reason: (
                self._mrg_response_callback(done_future, robot, service, why)
            )
        )

    def _mrg_response_callback(
        self,
        future,
        robot_name: str,
        service_name: str,
        reason: str,
    ) -> None:
        self._mrg_pending.pop(robot_name, None)
        try:
            response = future.result()
        except Exception as exc:
            self.file_logger.error(
                f"MRG graph service failed robot='{robot_name}' "
                f"service='{service_name}': {exc}"
            )
            return

        graph = response.graph
        if len(graph.keyframes) == 0 and len(graph.edges) == 0:
            return

        source = f'mrg_{sanitize_filename(robot_name)}'
        self._save_mrg_snapshot(
            graph,
            source,
            service_name,
            reason,
            robot_name,
        )

    def _source_files(self, source: str) -> SourceFiles:
        if source not in self._sources:
            source_dir = self.output_dir / sanitize_filename(source)
            source_dir.mkdir(parents=True, exist_ok=True)
            snapshot_file = (source_dir / 'snapshots.csv').open(
                'w',
                newline='',
                encoding='utf-8',
                buffering=1,
            )
            snapshot_writer = csv.writer(snapshot_file)
            snapshot_writer.writerow(self.SNAPSHOT_HEADER)
            self._sources[source] = SourceFiles(
                source_dir,
                snapshot_file,
                snapshot_writer,
            )
        return self._sources[source]

    def _save_swarm_snapshot(
        self,
        msg: PoseGraph,
        source: str,
        topic: str,
        reason: str,
    ) -> None:
        signature = self._swarm_signature(msg)
        files = self._source_files(source)
        if signature == files.last_signature:
            return

        files.sequence += 1
        files.last_signature = signature
        nodes_path = files.directory / f'nodes_{files.sequence:06d}.csv'
        edges_path = files.directory / f'edges_{files.sequence:06d}.csv'

        with nodes_path.open('w', newline='', encoding='utf-8') as nodes_file:
            writer = csv.writer(nodes_file)
            writer.writerow(self.SWARM_NODE_HEADER)
            for value in sorted(
                msg.values,
                key=lambda item: (
                    item.key.robot_id,
                    item.key.keyframe_id,
                ),
            ):
                robot_id = int(value.key.robot_id)
                keyframe_id = int(value.key.keyframe_id)
                stamp = self._swarm_keyframe_stamp(robot_id, keyframe_id)
                stamp_sec, stamp_nsec = stamp if stamp is not None else ('', '')
                writer.writerow(
                    [
                        files.sequence,
                        source,
                        robot_id,
                        keyframe_id,
                        stamp_sec,
                        stamp_nsec,
                        *self._pose_to_csv(value.pose),
                    ]
                )

        with edges_path.open('w', newline='', encoding='utf-8') as edges_file:
            writer = csv.writer(edges_file)
            writer.writerow(self.SWARM_EDGE_HEADER)
            for edge in sorted(
                msg.edges,
                key=lambda item: (
                    item.key_from.robot_id,
                    item.key_from.keyframe_id,
                    item.key_to.robot_id,
                    item.key_to.keyframe_id,
                ),
            ):
                writer.writerow(
                    [
                        files.sequence,
                        source,
                        self._swarm_edge_type(edge),
                        int(edge.key_from.robot_id),
                        int(edge.key_from.keyframe_id),
                        int(edge.key_to.robot_id),
                        int(edge.key_to.keyframe_id),
                        *self._pose_to_csv(edge.measurement),
                        *[
                            self._format_float(value)
                            for value in edge.noise_std
                        ],
                    ]
                )

        now_stamp = self.get_clock().now().to_msg()
        self._write_snapshot_row(
            files=files,
            stamp=now_stamp,
            source=source,
            source_type='swarm',
            reason=reason,
            robot_id=int(msg.robot_id),
            origin_robot_id=int(msg.origin_robot_id),
            robot_name='',
            topic_or_service=topic,
            node_count=len(msg.values),
            edge_count=len(msg.edges),
            nodes_path=nodes_path,
            edges_path=edges_path,
            signature=signature,
        )
        self.file_logger.info(
            f"Saved Swarm graph snapshot source='{source}' "
            f"seq={files.sequence} "
            f"robot_id={msg.robot_id} origin_robot_id={msg.origin_robot_id} "
            f"nodes={len(msg.values)} edges={len(msg.edges)}"
        )

    def _save_mrg_snapshot(
        self,
        graph,
        source: str,
        service_name: str,
        reason: str,
        requested_robot_name: str,
    ) -> None:
        signature = self._mrg_signature(graph)
        files = self._source_files(source)
        if signature == files.last_signature:
            return

        files.sequence += 1
        files.last_signature = signature
        nodes_path = files.directory / f'nodes_{files.sequence:06d}.csv'
        edges_path = files.directory / f'edges_{files.sequence:06d}.csv'

        graph_robot_name = graph.robot_name or requested_robot_name
        with nodes_path.open('w', newline='', encoding='utf-8') as nodes_file:
            writer = csv.writer(nodes_file)
            writer.writerow(self.MRG_NODE_HEADER)
            for keyframe in sorted(
                graph.keyframes,
                key=lambda item: (
                    item.robot_name,
                    item.odom_counter,
                    item.uuid_str,
                ),
            ):
                writer.writerow(
                    [
                        files.sequence,
                        source,
                        graph_robot_name,
                        keyframe.robot_name,
                        keyframe.uuid_str,
                        keyframe.slam_uuid_str,
                        int(keyframe.odom_counter),
                        int(keyframe.stamp.sec),
                        int(keyframe.stamp.nanosec),
                        self._format_float(keyframe.accum_distance),
                        bool(keyframe.first_keyframe),
                        bool(keyframe.static_keyframe),
                        *self._pose_to_csv(keyframe.estimate),
                    ]
                )

        with edges_path.open('w', newline='', encoding='utf-8') as edges_file:
            writer = csv.writer(edges_file)
            writer.writerow(self.MRG_EDGE_HEADER)
            for edge in sorted(
                graph.edges,
                key=lambda item: (
                    item.type,
                    item.from_uuid_str,
                    item.to_uuid_str,
                    item.uuid_str,
                ),
            ):
                writer.writerow(
                    [
                        files.sequence,
                        source,
                        edge.uuid_str,
                        edge.from_uuid_str,
                        edge.to_uuid_str,
                        self._mrg_edge_type(edge.type),
                        int(edge.type),
                        *self._pose_to_csv(edge.relative_pose),
                    ]
                )

        stamp = graph.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        self._write_snapshot_row(
            files=files,
            stamp=stamp,
            source=source,
            source_type='mrg',
            reason=reason,
            robot_id='',
            origin_robot_id='',
            robot_name=graph_robot_name,
            topic_or_service=service_name,
            node_count=len(graph.keyframes),
            edge_count=len(graph.edges),
            nodes_path=nodes_path,
            edges_path=edges_path,
            signature=signature,
        )
        self.file_logger.info(
            f"Saved MRG graph snapshot source='{source}' seq={files.sequence} "
            f"robot_name='{graph_robot_name}' "
            f"nodes={len(graph.keyframes)} edges={len(graph.edges)}"
        )

    def _write_snapshot_row(
        self,
        files: SourceFiles,
        stamp,
        source: str,
        source_type: str,
        reason: str,
        robot_id,
        origin_robot_id,
        robot_name: str,
        topic_or_service: str,
        node_count: int,
        edge_count: int,
        nodes_path: Path,
        edges_path: Path,
        signature: str,
    ) -> None:
        files.snapshot_writer.writerow(
            [
                files.sequence,
                f'{now_wall_time():.9f}',
                int(stamp.sec),
                int(stamp.nanosec),
                source,
                source_type,
                reason,
                robot_id,
                origin_robot_id,
                robot_name,
                topic_or_service,
                node_count,
                edge_count,
                str(nodes_path.relative_to(self.run_dir)),
                str(edges_path.relative_to(self.run_dir)),
                signature,
            ]
        )
        files.snapshot_file.flush()

    def _swarm_signature(self, msg: PoseGraph) -> str:
        nodes = [
            (
                int(value.key.robot_id),
                int(value.key.keyframe_id),
                *self._swarm_keyframe_stamp_for_signature(
                    int(value.key.robot_id),
                    int(value.key.keyframe_id),
                ),
                *self._pose_to_signature(value.pose),
            )
            for value in msg.values
        ]
        edges = [
            (
                int(edge.key_from.robot_id),
                int(edge.key_from.keyframe_id),
                int(edge.key_to.robot_id),
                int(edge.key_to.keyframe_id),
                *self._pose_to_signature(edge.measurement),
                *[self._rounded(value) for value in edge.noise_std],
            )
            for edge in msg.edges
        ]
        payload = {
            'robot_id': int(msg.robot_id),
            'origin_robot_id': int(msg.origin_robot_id),
            'nodes': sorted(nodes),
            'edges': sorted(edges),
        }
        return self._hash_payload(payload)

    def _swarm_keyframe_stamp(
        self,
        robot_id: int,
        keyframe_id: int,
    ) -> tuple[int, int] | None:
        return self._swarm_keyframe_stamps.get(
            (int(robot_id), int(keyframe_id))
        )

    def _swarm_keyframe_stamp_for_signature(
        self,
        robot_id: int,
        keyframe_id: int,
    ) -> tuple[int | None, int | None]:
        stamp = self._swarm_keyframe_stamp(robot_id, keyframe_id)
        if stamp is None:
            return None, None
        return stamp

    def _mrg_signature(self, graph) -> str:
        nodes = [
            (
                keyframe.uuid_str,
                keyframe.robot_name,
                int(keyframe.odom_counter),
                *self._pose_to_signature(keyframe.estimate),
            )
            for keyframe in graph.keyframes
        ]
        edges = [
            (
                edge.uuid_str,
                edge.from_uuid_str,
                edge.to_uuid_str,
                int(edge.type),
                *self._pose_to_signature(edge.relative_pose),
            )
            for edge in graph.edges
        ]
        payload = {
            'robot_name': graph.robot_name,
            'latest_keyframe_uuid_str': graph.latest_keyframe_uuid_str,
            'nodes': sorted(nodes),
            'edges': sorted(edges),
        }
        return self._hash_payload(payload)

    def _hash_payload(self, payload) -> str:
        encoded = json.dumps(
            payload,
            sort_keys=True,
            separators=(',', ':'),
        ).encode('utf-8')
        return hashlib.sha1(encoded).hexdigest()

    def _pose_to_signature(self, pose) -> list[float]:
        return [
            self._rounded(pose.position.x),
            self._rounded(pose.position.y),
            self._rounded(pose.position.z),
            self._rounded(pose.orientation.x),
            self._rounded(pose.orientation.y),
            self._rounded(pose.orientation.z),
            self._rounded(pose.orientation.w),
        ]

    def _pose_to_csv(self, pose) -> list[str]:
        return [
            self._format_float(pose.position.x),
            self._format_float(pose.position.y),
            self._format_float(pose.position.z),
            self._format_float(pose.orientation.x),
            self._format_float(pose.orientation.y),
            self._format_float(pose.orientation.z),
            self._format_float(pose.orientation.w),
        ]

    def _rounded(self, value: float) -> float:
        return round(float(value), self.signature_precision)

    def _format_float(self, value: float) -> str:
        return f'{float(value):.12g}'

    def _swarm_edge_type(self, edge) -> str:
        if int(edge.key_from.robot_id) != int(edge.key_to.robot_id):
            return 'inter_robot'
        keyframe_delta = abs(
            int(edge.key_from.keyframe_id)
            - int(edge.key_to.keyframe_id)
        )
        if keyframe_delta == 1:
            return 'odom'
        return 'loop'

    def _mrg_edge_type(self, edge_type: int) -> str:
        labels = {
            0: 'anchor',
            1: 'odom',
            2: 'loop',
        }
        return labels.get(int(edge_type), 'unknown')

    def close_files(self) -> None:
        for source, files in self._sources.items():
            files.snapshot_file.flush()
            files.snapshot_file.close()
            self.file_logger.info(
                f"Closed pose-graph source='{source}' "
                f"snapshots={files.sequence}"
            )
        self.file_logger.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseGraphRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
