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


@dataclass
class RobotTopicStats:
    message_type: str
    message_count: int = 0
    total_bytes: int = 0
    window_messages: int = 0
    window_bytes: int = 0
    peak_bandwidth_bps: float = 0.0


@dataclass(frozen=True)
class RobotAttribution:
    traffic_class: str
    source_robot: str
    peer_robot: str
    source_method: str = 'unknown'
    peer_method: str = 'unknown'


class SlamCommunicationRecorder(Node):
    """Estimate logical inter-robot SLAM communication."""

    SUMMARY_FILE = 'communication_summary.json'
    TOPICS_FILE = 'communication_topics.csv'
    ROBOT_TOPICS_FILE = 'communication_robot_topics.csv'
    TIMESERIES_FILE = 'communication_timeseries.csv'
    EVENTS_FILE = 'communication_events.csv'
    SERVICES_FILE_GLOB = 'communication_services*.csv'

    UNKNOWN_ROBOT = 'unknown'
    BROADCAST_ROBOT = 'broadcast'
    MULTIPLE_ROBOTS = 'multiple'

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
        'window_start',
        'window_end',
        'topic',
        'traffic_class',
        'source_robot',
        'peer_robot',
        'messages',
        'bytes',
        'bandwidth_Bps',
    ]

    EVENTS_HEADER = [
        'timestamp',
        'topic',
        'message_type',
        'traffic_class',
        'source_robot',
        'peer_robot',
        'message_bytes',
    ]

    ROBOT_TOPICS_HEADER = [
        'source_robot',
        'peer_robot',
        'topic',
        'traffic_class',
        'message_type',
        'message_count',
        'total_bytes',
        'total_MB',
        'average_message_size_bytes',
        'average_bandwidth_Bps',
        'peak_bandwidth_Bps',
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
        self._robot_topic_stats: dict[
            tuple[str, str, str, str],
            RobotTopicStats,
        ] = {}
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
            "Per-topic totals are recorded. Source robots are inferred from "
            "message payloads or topic namespaces when possible; destinations "
            "remain broadcast or unknown when the payload does not encode them."
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

    def _traffic_class_for_topic(self, topic: str) -> str:
        stripped = topic.strip('/')
        if 'global_descriptors' in stripped:
            return 'global_descriptors'
        if 'local_descriptors_request' in stripped:
            return 'local_descriptors_request'
        if 'local_descriptors' in stripped:
            return 'local_descriptors'
        if 'inter_robot_matches' in stripped:
            return 'inter_robot_matches'
        if 'inter_robot_loop_closure' in stripped:
            return 'inter_robot_loop_closure'
        if stripped.endswith('pose_graph') or '/pose_graph' in stripped:
            return 'pose_graph'
        if 'optimized_estimates' in stripped:
            return 'optimized_estimates'
        if 'heartbeat' in stripped:
            return 'heartbeat'
        if 'get_pose_graph' in stripped:
            return 'get_pose_graph'
        if 'mrg_slam' in stripped:
            return 'mrg_pose_broadcast'
        return 'topic'

    def _topic_robot(self, topic: str) -> str | None:
        parts = [part for part in topic.strip('/').split('/') if part]
        if not parts:
            return None
        first = parts[0]
        robot_names = {name.strip('/') for name in self.robot_names}
        if first in robot_names:
            return first
        if re.fullmatch(r'r\d+', first):
            return first
        return None

    def _robot_name_from_value(self, value: object) -> str | None:
        if value is None or isinstance(value, bool):
            return None
        if isinstance(value, str):
            stripped = value.strip().strip('/')
            if not stripped:
                return None
            if stripped in self.robot_names:
                return stripped
            match = re.fullmatch(r'(?:robot|r)?(\d+)', stripped)
            if match:
                return self._robot_name_from_value(int(match.group(1)))
            return stripped
        try:
            robot_id = int(value)
        except (TypeError, ValueError):
            return None
        if robot_id < 0:
            return None
        candidate = f'r{robot_id}'
        if candidate in self.robot_names:
            return candidate
        if robot_id < len(self.robot_names):
            return self.robot_names[robot_id].strip('/')
        return candidate

    def _message_field_names(self, value: object) -> list[str]:
        if hasattr(value, 'get_fields_and_field_types'):
            try:
                return list(value.get_fields_and_field_types().keys())
            except Exception:
                return []
        slots = getattr(value, '__slots__', [])
        names = []
        for slot in slots:
            name = str(slot)
            if name.startswith('_'):
                name = name[1:]
            if name:
                names.append(name)
        return names

    def _is_sequence_value(self, value: object) -> bool:
        return (
            hasattr(value, '__iter__')
            and not isinstance(value, (str, bytes, bytearray, dict))
        )

    def _unique_robot_from_values(self, values: list[str]) -> tuple[str, str]:
        unique = sorted({value for value in values if value})
        if len(unique) == 1:
            return unique[0], 'payload'
        if len(unique) > 1:
            return self.MULTIPLE_ROBOTS, 'payload_multiple'
        return self.UNKNOWN_ROBOT, 'unknown'

    def _direct_robot_field(self, msg: object, names: list[str]) -> tuple[str, str]:
        for name in names:
            if not hasattr(msg, name):
                continue
            robot = self._robot_name_from_value(getattr(msg, name))
            if robot:
                return robot, f'payload.{name}'
        return self.UNKNOWN_ROBOT, 'unknown'

    def _robots_from_sequence_field(
        self,
        msg: object,
        name: str,
    ) -> list[str]:
        if not hasattr(msg, name):
            return []
        value = getattr(msg, name)
        if not self._is_sequence_value(value):
            robot = self._robot_name_from_value(value)
            return [robot] if robot else []
        robots = []
        for item in list(value)[:64]:
            robot = self._robot_name_from_value(item)
            if robot:
                robots.append(robot)
        return robots

    def _nested_robot_values(
        self,
        value: object,
        field_names: set[str],
        depth: int = 0,
    ) -> list[str]:
        if depth > 3 or value is None:
            return []
        if isinstance(value, (str, bytes, bytearray, bool, int, float)):
            return []
        if self._is_sequence_value(value):
            robots: list[str] = []
            for item in list(value)[:64]:
                robots.extend(
                    self._nested_robot_values(item, field_names, depth + 1)
                )
            return robots

        skip_fields = {
            'data',
            'pointcloud',
            'image',
            'rgb',
            'depth',
            'descriptor',
            'transform',
            'pose',
            'covariance',
            'values',
            'edges',
            'factors',
            'estimates',
            'gps_values',
        }
        robots = []
        for field_name in self._message_field_names(value):
            if field_name in skip_fields:
                continue
            try:
                field_value = getattr(value, field_name)
            except Exception:
                continue
            if field_name in field_names:
                if self._is_sequence_value(field_value):
                    for item in list(field_value)[:64]:
                        robot = self._robot_name_from_value(item)
                        if robot:
                            robots.append(robot)
                else:
                    robot = self._robot_name_from_value(field_value)
                    if robot:
                        robots.append(robot)
                continue
            robots.extend(
                self._nested_robot_values(field_value, field_names, depth + 1)
            )
        return robots

    def _infer_robot_identity(
        self,
        topic: str,
        msg: object,
    ) -> RobotAttribution:
        traffic_class = self._traffic_class_for_topic(topic)
        topic_robot = self._topic_robot(topic)
        source_robot = self.UNKNOWN_ROBOT
        peer_robot = self.UNKNOWN_ROBOT
        source_method = 'unknown'
        peer_method = 'unknown'

        if traffic_class == 'inter_robot_loop_closure':
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['robot0_id'],
            )
            peer_robot, peer_method = self._direct_robot_field(
                msg,
                ['robot1_id'],
            )
        elif traffic_class == 'inter_robot_matches':
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['robot_id'],
            )
            match_robots = self._nested_robot_values(
                getattr(msg, 'matches', []),
                {'robot0_id', 'robot1_id'},
            )
            peer_candidates = [
                robot for robot in match_robots if robot != source_robot
            ]
            peer_robot, peer_method = self._unique_robot_from_values(
                peer_candidates
            )
        elif traffic_class in ('local_descriptors', 'global_descriptors'):
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['robot_id'],
            )
            if source_robot == self.UNKNOWN_ROBOT:
                descriptor_robots = self._nested_robot_values(
                    msg,
                    {'robot_id'},
                )
                source_robot, source_method = self._unique_robot_from_values(
                    descriptor_robots
                )
            match_robots = self._robots_from_sequence_field(
                msg,
                'matches_robot_id',
            )
            peer_robot, peer_method = self._unique_robot_from_values(match_robots)
        elif traffic_class == 'pose_graph':
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['robot_id', 'origin_robot_id'],
            )
            connected = getattr(getattr(msg, 'connected_robots', None), 'ids', [])
            peer_robot, peer_method = self._unique_robot_from_values(
                [
                    robot
                    for robot in (
                        self._robot_name_from_value(value) for value in connected
                    )
                    if robot
                ]
            )
        elif traffic_class == 'optimized_estimates':
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['robot_id', 'source_robot_id', 'sender_id', 'origin_robot_id'],
            )
        elif traffic_class == 'heartbeat':
            if topic_robot:
                source_robot = topic_robot
                source_method = 'topic_namespace'
                peer_robot = self.BROADCAST_ROBOT
                peer_method = 'heartbeat_topic'
            elif hasattr(msg, 'data'):
                robot = self._robot_name_from_value(getattr(msg, 'data'))
                if robot:
                    source_robot = robot
                    source_method = 'payload.data'
                    peer_robot = self.BROADCAST_ROBOT
                    peer_method = 'heartbeat_topic'
        elif traffic_class in ('get_pose_graph', 'local_descriptors_request'):
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['robot_id', 'source_robot_id', 'sender_id', 'origin_robot_id'],
            )
            match_robots = self._robots_from_sequence_field(
                msg,
                'matches_robot_id',
            )
            peer_robot, peer_method = self._unique_robot_from_values(match_robots)
        else:
            source_robot, source_method = self._direct_robot_field(
                msg,
                ['source_robot', 'source_robot_id', 'sender_robot',
                 'sender_id', 'robot_name', 'robot_id', 'origin_robot_id'],
            )

        if source_robot in (self.UNKNOWN_ROBOT, self.MULTIPLE_ROBOTS):
            nested = self._nested_robot_values(
                msg,
                {
                    'source_robot',
                    'source_robot_id',
                    'sender_robot',
                    'sender_id',
                    'robot_name',
                    'robot_id',
                    'origin_robot_id',
                },
            )
            nested_robot, nested_method = self._unique_robot_from_values(nested)
            if nested_robot != self.UNKNOWN_ROBOT:
                source_robot = nested_robot
                source_method = nested_method

        if source_robot == self.UNKNOWN_ROBOT and topic_robot:
            source_robot = topic_robot
            source_method = 'topic_namespace'

        if peer_robot == self.UNKNOWN_ROBOT:
            if topic_robot and topic_robot != source_robot:
                peer_robot = topic_robot
                peer_method = 'topic_namespace'
            elif topic.startswith('/cslam/'):
                peer_robot = self.BROADCAST_ROBOT
                peer_method = 'global_topic'

        if peer_robot == source_robot and topic.startswith('/cslam/'):
            peer_robot = self.BROADCAST_ROBOT
            peer_method = 'global_topic'

        return RobotAttribution(
            traffic_class=traffic_class,
            source_robot=source_robot or self.UNKNOWN_ROBOT,
            peer_robot=peer_robot or self.UNKNOWN_ROBOT,
            source_method=source_method,
            peer_method=peer_method,
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

        attribution = self._infer_robot_identity(topic, msg)
        robot_key = (
            topic,
            attribution.traffic_class,
            attribution.source_robot,
            attribution.peer_robot,
        )
        robot_stats = self._robot_topic_stats.setdefault(
            robot_key,
            RobotTopicStats(message_type_name),
        )
        robot_stats.message_count += 1
        robot_stats.total_bytes += message_bytes
        robot_stats.window_messages += 1
        robot_stats.window_bytes += message_bytes

        if self._events_writer is not None:
            self._events_writer.writerow(
                [
                    f'{now_wall_time():.9f}',
                    topic,
                    message_type_name,
                    attribution.traffic_class,
                    attribution.source_robot,
                    attribution.peer_robot,
                    message_bytes,
                ]
            )

    def _sample_timer_callback(self) -> None:
        now = now_wall_time()
        window_start = self._last_sample_time
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
            stats.window_messages = 0
            stats.window_bytes = 0

        for (
            topic,
            traffic_class,
            source_robot,
            peer_robot,
        ), stats in sorted(self._robot_topic_stats.items()):
            if stats.window_messages == 0 and stats.window_bytes == 0:
                continue
            bandwidth_bps = float(stats.window_bytes) / elapsed
            stats.peak_bandwidth_bps = max(
                stats.peak_bandwidth_bps,
                bandwidth_bps,
            )
            self._timeseries_writer.writerow(
                [
                    f'{now:.9f}',
                    f'{window_start:.9f}',
                    f'{now:.9f}',
                    topic,
                    traffic_class,
                    source_robot,
                    peer_robot,
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
        robot_tx, robot_rx, robot_unknown, unattributed_bytes = (
            self._robot_attribution_summary()
        )

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
            'per_robot_tx_estimate': robot_tx,
            'per_robot_rx_estimate': robot_rx,
            'per_robot_unknown_bytes': robot_unknown,
            'unattributed_bytes': unattributed_bytes,
            'unattributed_MB': self._bytes_to_mb(unattributed_bytes),
            'attribution_notes': [
                (
                    'source_robot is inferred from Swarm payload fields such '
                    'as robot_id/origin_robot_id when present, otherwise from '
                    'the topic namespace when possible.'
                ),
                (
                    'peer_robot is only filled when a destination/peer is '
                    'explicitly inferable. Global Swarm topics usually use '
                    'peer_robot=broadcast.'
                ),
                (
                    'per_robot_tx_estimate is a logical serialized payload '
                    'estimate grouped by inferred source_robot; '
                    'per_robot_rx_estimate is only populated for concrete '
                    'peer_robot values and is therefore a lower bound.'
                ),
            ],
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

        with (self.output_dir / self.ROBOT_TOPICS_FILE).open(
            'w',
            newline='',
            encoding='utf-8',
        ) as robot_topics_file:
            writer = csv.writer(robot_topics_file)
            writer.writerow(self.ROBOT_TOPICS_HEADER)
            for (
                topic,
                traffic_class,
                source_robot,
                peer_robot,
            ), stats in sorted(
                self._robot_topic_stats.items(),
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
                        source_robot,
                        peer_robot,
                        topic,
                        traffic_class,
                        stats.message_type,
                        stats.message_count,
                        stats.total_bytes,
                        f'{self._bytes_to_mb(stats.total_bytes):.9f}',
                        f'{avg_message_size:.9f}',
                        f'{average_topic_bps:.9f}',
                        f'{stats.peak_bandwidth_bps:.9f}',
                    ]
                )

    def _is_concrete_robot(self, value: str) -> bool:
        return value not in (
            '',
            self.UNKNOWN_ROBOT,
            self.BROADCAST_ROBOT,
            self.MULTIPLE_ROBOTS,
        )

    def _robot_attribution_summary(
        self,
    ) -> tuple[dict[str, int], dict[str, int], dict[str, int], int]:
        robot_tx = {name.strip('/'): 0 for name in self.robot_names}
        robot_rx = {name.strip('/'): 0 for name in self.robot_names}
        robot_unknown = {name.strip('/'): 0 for name in self.robot_names}
        unattributed_bytes = 0

        for (
            _topic,
            _traffic_class,
            source_robot,
            peer_robot,
        ), stats in self._robot_topic_stats.items():
            if self._is_concrete_robot(source_robot):
                robot_tx[source_robot] = (
                    robot_tx.get(source_robot, 0) + stats.total_bytes
                )
                if not self._is_concrete_robot(peer_robot):
                    robot_unknown[source_robot] = (
                        robot_unknown.get(source_robot, 0) + stats.total_bytes
                    )
            else:
                unattributed_bytes += stats.total_bytes

            if self._is_concrete_robot(peer_robot):
                robot_rx[peer_robot] = (
                    robot_rx.get(peer_robot, 0) + stats.total_bytes
                )

        return robot_tx, robot_rx, robot_unknown, unattributed_bytes

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
