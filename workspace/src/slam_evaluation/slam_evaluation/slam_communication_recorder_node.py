#!/usr/bin/env python3

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
from pathlib import Path
import re
from typing import Pattern, TextIO

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from slam_evaluation.result_utils import (
    FileEventLogger,
    make_run_directory,
    now_wall_time,
    parse_csv,
)


@dataclass
class TopicStats:
    message_type: str
    message_count: int = 0
    total_bytes: int = 0
    window_messages: int = 0
    window_bytes: int = 0
    peak_bandwidth_bps: float = 0.0


class SlamCommunicationRecorder(Node):
    """Estimate logical inter-robot SLAM communication."""

    SUMMARY_FILE = 'communication_summary.json'
    TOPICS_FILE = 'communication_topics.csv'
    TIMESERIES_FILE = 'communication_timeseries.csv'
    EVENTS_FILE = 'communication_events.csv'
    SERVICES_FILE_GLOB = 'communication_services*.csv'

    TOPICS_HEADER = [
        'topic',
        'message_type',
        'message_count',
        'total_bytes',
        'total_MB',
        'average_message_size_bytes',
        'average_bandwidth_Bps',
        'peak_bandwidth_Bps',
    ]

    TIMESERIES_HEADER = [
        'timestamp',
        'topic',
        'messages',
        'bytes',
        'bandwidth_Bps',
    ]

    EVENTS_HEADER = [
        'timestamp',
        'topic',
        'message_type',
        'bytes',
    ]

    MRG_SERVICE_PATTERN = re.compile(
        r'^/(?:[^/]+/)?mrg_slam/(?:publish_graph|request_graph)$'
    )

    def __init__(self) -> None:
        super().__init__('slam_communication_recorder')

        self.declare_parameter('slam_type', 'auto')
        self.declare_parameter('robot_names', '')
        self.declare_parameter('results_root', '')
        self.declare_parameter('run_type', 'run')
        self.declare_parameter(
            'run_id',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('output_dir', '')
        self.declare_parameter('include_topic_regexes', '')
        self.declare_parameter('exclude_topic_regexes', '')
        self.declare_parameter('sample_period_sec', 1.0)
        self.declare_parameter('discovery_period_sec', 2.0)
        self.declare_parameter('write_raw_events', False)

        self.slam_type = str(self.get_parameter('slam_type').value).lower()
        if self.slam_type not in ('auto', 'mrg', 'swarm'):
            self.get_logger().warn(
                "Parameter 'slam_type' must be auto, mrg, or swarm; "
                f"got '{self.slam_type}', using auto."
            )
            self.slam_type = 'auto'

        self.robot_names = self._resolve_robot_names()
        self.run_dir = self._resolve_run_dir()
        self.output_dir = self.run_dir / 'communication'
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.file_logger = FileEventLogger(
            self,
            self.output_dir / 'communication_recorder.log',
        )

        self._communication_subscriptions = {}
        self._topic_stats: dict[str, TopicStats] = {}
        self._topic_type_failures: set[str] = set()
        self._serialization_failures: set[str] = set()
        self._detected_mrg_services: set[str] = set()
        self._warnings: set[str] = set()

        self.include_patterns = self._compile_patterns(
            [
                *self._default_include_regexes(),
                *self._parse_regex_list(
                    self.get_parameter('include_topic_regexes').value
                ),
            ],
            'include_topic_regexes',
        )
        self.exclude_patterns = self._compile_patterns(
            [
                *self._default_exclude_regexes(),
                *self._parse_regex_list(
                    self.get_parameter('exclude_topic_regexes').value
                ),
            ],
            'exclude_topic_regexes',
        )

        self.sample_period_sec = max(
            0.1,
            float(self.get_parameter('sample_period_sec').value),
        )
        discovery_period_sec = max(
            0.5,
            float(self.get_parameter('discovery_period_sec').value),
        )
        self.write_raw_events = self._as_bool(
            self.get_parameter('write_raw_events').value
        )

        self._start_time = now_wall_time()
        self._last_sample_time = self._start_time
        self._peak_bandwidth_bps = 0.0

        self._timeseries_file = (self.output_dir / self.TIMESERIES_FILE).open(
            'w',
            newline='',
            encoding='utf-8',
            buffering=1,
        )
        self._timeseries_writer = csv.writer(self._timeseries_file)
        self._timeseries_writer.writerow(self.TIMESERIES_HEADER)

        self._events_file: TextIO | None = None
        self._events_writer: csv.writer | None = None
        if self.write_raw_events:
            self._events_file = (self.output_dir / self.EVENTS_FILE).open(
                'w',
                newline='',
                encoding='utf-8',
                buffering=1,
            )
            self._events_writer = csv.writer(self._events_file)
            self._events_writer.writerow(self.EVENTS_HEADER)

        self._add_baseline_notes()
        self._discover_topics_and_services()
        self.create_timer(
            discovery_period_sec,
            self._discover_topics_and_services,
        )
        self.create_timer(self.sample_period_sec, self._sample_timer_callback)

        self.file_logger.info(
            "Communication recorder ready "
            f"slam_type='{self.slam_type}' robots={self.robot_names} "
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

    def _resolve_run_dir(self) -> Path:
        configured_output_dir = str(
            self.get_parameter('output_dir').value
        ).strip()
        if configured_output_dir:
            run_dir = Path(configured_output_dir).expanduser().resolve()
            run_dir.mkdir(parents=True, exist_ok=True)
            return run_dir

        return make_run_directory(
            str(self.get_parameter('results_root').value),
            str(self.get_parameter('run_type').value),
            str(self.get_parameter('run_id').value),
        )

    def _as_bool(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('true', '1', 'yes', 'on')

    def _parse_regex_list(self, value: object) -> list[str]:
        if value is None:
            return []
        if isinstance(value, (list, tuple)):
            raw_items = value
        else:
            raw_items = str(value).split(',')
        return [str(item).strip() for item in raw_items if str(item).strip()]

    def _compile_patterns(
        self,
        values: list[str],
        parameter_name: str,
    ) -> list[Pattern[str]]:
        patterns: list[Pattern[str]] = []
        for value in values:
            try:
                patterns.append(re.compile(value))
            except re.error as exc:
                self._record_warning(
                    f"Ignoring invalid regex from {parameter_name}: "
                    f"'{value}' ({exc})"
                )
        return patterns

    def _default_include_regexes(self) -> list[str]:
        regexes: list[str] = []
        if self.slam_type in ('auto', 'swarm'):
            regexes.extend(
                [
                    r'^/cslam/global_descriptors$',
                    r'^/cslam/inter_robot_matches$',
                    r'^/cslam/local_descriptors$',
                    r'^/cslam/inter_robot_loop_closure$',
                    r'^/cslam/pose_graph$',
                    (
                        r'^/[^/]+/cslam/'
                        r'(?:local_descriptors_request|get_pose_graph|'
                        r'optimized_estimates|heartbeat)$'
                    ),
                ]
            )
            for robot_name in self.robot_names:
                escaped = re.escape(robot_name.strip('/'))
                regexes.extend(
                    [
                        f'^/{escaped}/cslam/local_descriptors_request$',
                        f'^/{escaped}/cslam/get_pose_graph$',
                        f'^/{escaped}/cslam/optimized_estimates$',
                        f'^/{escaped}/cslam/heartbeat$',
                    ]
                )

        if self.slam_type in ('auto', 'mrg'):
            regexes.extend(
                [
                    r'^/mrg_slam/odom_broadcast$',
                    r'^/mrg_slam/slam_pose_broadcast$',
                ]
            )
            for robot_name in self.robot_names:
                escaped = re.escape(robot_name.strip('/'))
                regexes.extend(
                    [
                        f'^/{escaped}/mrg_slam/odom_broadcast$',
                        f'^/{escaped}/mrg_slam/slam_pose_broadcast$',
                    ]
                )
        return regexes

    def _default_exclude_regexes(self) -> list[str]:
        return [
            r'^/(?:tf|tf_static|clock|rosout|parameter_events)$',
            r'/(?:ground_truth|scan_matching_odometry|prefiltering|'
            r'floor_detection|velodyne_points|imu|gps|navsat)(?:/|$)',
            r'/(?:sensors|camera|image|depth|camera_info)(?:/|$)',
            r'/(?:markers|markers_covariance|rviz|cslam_rviz)(?:/|$)',
            r'/(?:map_points|map_points_service|'
            r'other_robots_removed_points)$',
            r'/(?:keyframe_odom|keyframe_data|intra_robot_loop_closure)$',
            r'/(?:current_pose_estimate|optimizer_state|reference_frames)$',
            r'/(?:debug_optimization_result|log_info|log_matches)$',
            r'/(?:print_current_estimates|local_keyframe_match|'
            r'get_current_neighbors|current_neighbors)$',
            r'^/cslam/viz/',
            r'/cslam/viz/',
            r'^/[^/]+/slam/pose$',
        ]

    def _add_baseline_notes(self) -> None:
        self._record_warning(
            "This recorder estimates logical serialized ROS topic payloads "
            "for collaborative SLAM exchange; it does not measure physical "
            "DDS or network bandwidth."
        )
        self._record_warning(
            "Per-topic totals are recorded; source and destination robots are "
            "not inferred in this passive recorder."
        )
        if self.slam_type in ('auto', 'mrg'):
            self._record_warning(
                "MRG graph exchange uses PublishGraph services carrying "
                "keyframes, point clouds and edges. Passive topic recording "
                "does not measure those service request/response payloads, so "
                "topic-based totals are a lower bound on MRG communication."
            )

    def _discover_topics_and_services(self) -> None:
        try:
            topics_and_types = self.get_topic_names_and_types()
        except Exception as exc:
            self._record_warning(f'Failed to discover topics: {exc}')
            return

        for topic, message_types in topics_and_types:
            if topic in self._communication_subscriptions:
                continue
            if not message_types:
                continue
            if not self._topic_is_selected(topic):
                continue
            self._subscribe_to_topic(topic, message_types[0])

        self._discover_mrg_services()
        self._write_summary_and_topics()

    def _topic_is_selected(self, topic: str) -> bool:
        if not any(pattern.search(topic) for pattern in self.include_patterns):
            return False
        return not any(
            pattern.search(topic) for pattern in self.exclude_patterns
        )

    def _subscribe_to_topic(self, topic: str, message_type_name: str) -> None:
        try:
            message_class = get_message(message_type_name)
        except Exception as exc:
            key = f'{topic}:{message_type_name}'
            if key not in self._topic_type_failures:
                self._topic_type_failures.add(key)
                self._record_warning(
                    f"Could not import message type '{message_type_name}' "
                    f"for topic '{topic}': {exc}"
                )
            return

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        try:
            subscription = self.create_subscription(
                message_class,
                topic,
                lambda msg, top=topic, msg_type=message_type_name: (
                    self._message_callback(msg, top, msg_type)
                ),
                qos,
            )
        except Exception as exc:
            self._record_warning(
                f"Could not subscribe to topic '{topic}' "
                f"({message_type_name}): {exc}"
            )
            return

        self._communication_subscriptions[topic] = subscription
        self._topic_stats.setdefault(topic, TopicStats(message_type_name))
        self.file_logger.info(
            f"Recording communication topic '{topic}' "
            f"type='{message_type_name}'"
        )

    def _discover_mrg_services(self) -> None:
        if self.slam_type not in ('auto', 'mrg'):
            return
        try:
            services_and_types = self.get_service_names_and_types()
        except Exception as exc:
            self._record_warning(f'Failed to discover services: {exc}')
            return

        for service_name, service_types in services_and_types:
            if not self.MRG_SERVICE_PATTERN.search(service_name):
                continue
            if service_name in self._detected_mrg_services:
                continue
            self._detected_mrg_services.add(service_name)
            types_text = ','.join(service_types)
            self._record_warning(
                "Detected relevant MRG graph-exchange service "
                f"'{service_name}' ({types_text}). Service payloads are not "
                "passively measured by this topic recorder."
            )

    def _message_callback(
        self,
        msg,
        topic: str,
        message_type_name: str,
    ) -> None:
        try:
            message_bytes = len(serialize_message(msg))
        except Exception as exc:
            key = f'{topic}:{message_type_name}'
            if key not in self._serialization_failures:
                self._serialization_failures.add(key)
                self._record_warning(
                    f"Failed to serialize messages from topic '{topic}' "
                    f"({message_type_name}): {exc}"
                )
            return

        stats = self._topic_stats.setdefault(
            topic,
            TopicStats(message_type_name),
        )
        stats.message_count += 1
        stats.total_bytes += message_bytes
        stats.window_messages += 1
        stats.window_bytes += message_bytes

        if self._events_writer is not None:
            self._events_writer.writerow(
                [
                    f'{now_wall_time():.9f}',
                    topic,
                    message_type_name,
                    message_bytes,
                ]
            )

    def _sample_timer_callback(self) -> None:
        now = now_wall_time()
        elapsed = max(now - self._last_sample_time, 1e-9)
        self._last_sample_time = now
        total_window_bytes = 0

        for topic, stats in sorted(self._topic_stats.items()):
            if stats.window_messages == 0 and stats.window_bytes == 0:
                continue
            bandwidth_bps = float(stats.window_bytes) / elapsed
            stats.peak_bandwidth_bps = max(
                stats.peak_bandwidth_bps,
                bandwidth_bps,
            )
            total_window_bytes += stats.window_bytes
            self._timeseries_writer.writerow(
                [
                    f'{now:.9f}',
                    topic,
                    stats.window_messages,
                    stats.window_bytes,
                    f'{bandwidth_bps:.9f}',
                ]
            )
            stats.window_messages = 0
            stats.window_bytes = 0

        self._peak_bandwidth_bps = max(
            self._peak_bandwidth_bps,
            float(total_window_bytes) / elapsed,
        )
        self._write_summary_and_topics()

    def _write_summary_and_topics(self) -> None:
        duration = max(now_wall_time() - self._start_time, 0.0)
        total_messages = sum(
            stats.message_count for stats in self._topic_stats.values()
        )
        total_bytes = sum(
            stats.total_bytes for stats in self._topic_stats.values()
        )
        average_bps = float(total_bytes) / duration if duration > 0.0 else 0.0

        service_metrics, service_files = self._load_service_metrics()
        service_total_bytes = service_metrics['service_total_bytes']
        combined_total_bytes = total_bytes + service_total_bytes

        warnings = sorted(self._warnings)
        if total_messages == 0:
            warnings.append(
                "No matching inter-robot SLAM topic messages have been "
                "observed yet."
            )
        if self.slam_type in ('auto', 'mrg') and not service_files:
            warnings.append(
                "No MRG publish_graph service payload metrics were found. "
                "Ensure mrg_slam sets communication_output_dir to this run's "
                "communication folder."
            )

        summary = {
            'metric_name': 'estimated logical inter-robot communication',
            'slam_type': self.slam_type,
            'robot_names': self.robot_names,
            'duration_sec': duration,
            'total_messages': total_messages,
            'total_bytes': total_bytes,
            'total_MB': self._bytes_to_mb(total_bytes),
            'topic_total_bytes': total_bytes,
            'topic_total_MB': self._bytes_to_mb(total_bytes),
            'service_total_bytes': service_total_bytes,
            'service_total_MB': self._bytes_to_mb(service_total_bytes),
            'combined_total_bytes': combined_total_bytes,
            'combined_total_MB': self._bytes_to_mb(combined_total_bytes),
            'service_request_bytes': service_metrics['service_request_bytes'],
            'service_response_bytes': service_metrics['service_response_bytes'],
            'service_event_count': service_metrics['service_event_count'],
            'service_keyframes': service_metrics['service_keyframes'],
            'service_edges': service_metrics['service_edges'],
            'service_cloud_bytes': service_metrics['service_cloud_bytes'],
            'average_bandwidth_Bps': average_bps,
            'average_bandwidth_MBps': self._bytes_to_mb(average_bps),
            'peak_bandwidth_Bps': self._peak_bandwidth_bps,
            'peak_bandwidth_MBps': self._bytes_to_mb(
                self._peak_bandwidth_bps
            ),
            'measured_topic_count': len(self._topic_stats),
            'subscribed_topics': sorted(self._topic_stats),
            'detected_relevant_services': sorted(self._detected_mrg_services),
            'notes': [
                (
                    'Bandwidth uses wall-clock time and serialized ROS '
                    'message size.'
                ),
                (
                    'This is logical ROS payload, not physical network usage; '
                    'DDS transport, shared memory, loopback, QoS, retries and '
                    'network hardware are outside this metric.'
                ),
            ],
            'warnings': warnings,
        }
        (self.output_dir / self.SUMMARY_FILE).write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )

        with (self.output_dir / self.TOPICS_FILE).open(
            'w',
            newline='',
            encoding='utf-8',
        ) as topics_file:
            writer = csv.writer(topics_file)
            writer.writerow(self.TOPICS_HEADER)
            for topic, stats in sorted(
                self._topic_stats.items(),
                key=lambda item: (-item[1].total_bytes, item[0]),
            ):
                avg_message_size = (
                    float(stats.total_bytes) / float(stats.message_count)
                    if stats.message_count > 0
                    else 0.0
                )
                average_topic_bps = (
                    float(stats.total_bytes) / duration
                    if duration > 0.0
                    else 0.0
                )
                writer.writerow(
                    [
                        topic,
                        stats.message_type,
                        stats.message_count,
                        stats.total_bytes,
                        f'{self._bytes_to_mb(stats.total_bytes):.9f}',
                        f'{avg_message_size:.9f}',
                        f'{average_topic_bps:.9f}',
                        f'{stats.peak_bandwidth_bps:.9f}',
                    ]
                )

    def _load_service_metrics(self) -> tuple[dict[str, int], list[Path]]:
        service_files = sorted(self.output_dir.glob(self.SERVICES_FILE_GLOB))
        totals = {
            'service_event_count': 0,
            'service_request_bytes': 0,
            'service_response_bytes': 0,
            'service_total_bytes': 0,
            'service_keyframes': 0,
            'service_edges': 0,
            'service_cloud_bytes': 0,
        }
        if not service_files:
            return totals, []

        def _safe_int(value: object) -> int:
            try:
                return int(float(value))
            except (TypeError, ValueError):
                return 0

        for path in service_files:
            try:
                with path.open('r', encoding='utf-8') as handle:
                    reader = csv.DictReader(handle)
                    for row in reader:
                        request_bytes = _safe_int(row.get('request_bytes'))
                        response_bytes = _safe_int(row.get('response_bytes'))
                        total_bytes = _safe_int(row.get('total_bytes'))
                        if total_bytes == 0:
                            total_bytes = request_bytes + response_bytes

                        totals['service_event_count'] += 1
                        totals['service_request_bytes'] += request_bytes
                        totals['service_response_bytes'] += response_bytes
                        totals['service_total_bytes'] += total_bytes
                        totals['service_keyframes'] += _safe_int(
                            row.get('response_keyframes')
                        )
                        totals['service_edges'] += _safe_int(
                            row.get('response_edges')
                        )
                        totals['service_cloud_bytes'] += _safe_int(
                            row.get('response_cloud_bytes')
                        )
            except Exception as exc:
                self._record_warning(
                    f"Failed to read communication service metrics from "
                    f"{path}: {exc}"
                )

        return totals, service_files

    def _record_warning(self, message: str) -> None:
        if message in self._warnings:
            return
        self._warnings.add(message)
        if hasattr(self, 'file_logger'):
            self.file_logger.warn(message)
        else:
            self.get_logger().warn(message)

    def _bytes_to_mb(self, value: float | int) -> float:
        return float(value) / 1_000_000.0

    def close(self) -> None:
        self._write_summary_and_topics()
        self._timeseries_file.close()
        if self._events_file is not None:
            self._events_file.close()
        self.file_logger.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamCommunicationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
