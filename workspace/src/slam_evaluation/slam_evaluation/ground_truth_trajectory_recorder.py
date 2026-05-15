#!/usr/bin/env python3

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, TextIO

from geometry_msgs.msg import PoseStamped
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from slam_evaluation.result_utils import (
    FileEventLogger,
    make_run_directory,
    now_wall_time,
    parse_csv,
    sanitize_filename,
)


class GroundTruthTrajectoryRecorder(Node):
    """Record ground-truth trajectories to per-robot CSV files."""

    HEADER = [
        'ros_time_sec',
        'ros_time_nsec',
        'wall_time_unix_sec',
        'robot_name',
        'frame_id',
        'position_x',
        'position_y',
        'position_z',
        'orientation_x',
        'orientation_y',
        'orientation_z',
        'orientation_w',
    ]

    def __init__(self) -> None:
        super().__init__('ground_truth_trajectory_recorder')

        self.declare_parameter('robot_names', '')
        self.declare_parameter(
            'pose_topic_template',
            '/{robot_name}/ground_truth/pose',
        )
        self.declare_parameter('results_root', '')
        self.declare_parameter('run_type', 'run')
        self.declare_parameter(
            'run_id',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('flush_every_n', 1)

        self.pose_topic_template = str(
            self.get_parameter('pose_topic_template').value
        )
        self.flush_every_n = max(
            1,
            int(self.get_parameter('flush_every_n').value),
        )
        self.run_dir = make_run_directory(
            str(self.get_parameter('results_root').value),
            str(self.get_parameter('run_type').value),
            str(self.get_parameter('run_id').value),
        )
        self.output_dir = self.run_dir / 'ground_truth'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file_logger = FileEventLogger(
            self,
            self.output_dir / 'ground_truth_recorder.log',
        )

        robot_names = self._resolve_robot_names()
        self._files: Dict[str, TextIO] = {}
        self._writers: Dict[str, csv.writer] = {}
        self._counts: Dict[str, int] = {
            robot_name: 0 for robot_name in robot_names
        }
        self._subscriptions = []

        for robot_name in robot_names:
            topic = self.pose_topic_template.format(robot_name=robot_name)
            self._subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, robot=robot_name: (
                        self._pose_callback(msg, robot)
                    ),
                    100,
                )
            )
            self.file_logger.info(
                f"Recording ground truth for robot='{robot_name}' "
                f"from topic='{topic}' "
                f"to '{self._csv_path(robot_name)}'"
            )

    def _resolve_robot_names(self) -> list[str]:
        raw_value = self.get_parameter('robot_names').value
        robot_names = parse_csv(raw_value)
        if robot_names:
            return robot_names

        namespace = self.get_namespace().strip('/')
        if namespace:
            return [namespace]

        return ['r0']

    def _csv_path(self, robot_name: str) -> Path:
        return (
            self.output_dir
            / f'ground_truth_{sanitize_filename(robot_name)}.csv'
        )

    def _writer_for_robot(self, robot_name: str) -> csv.writer:
        if robot_name not in self._writers:
            file_path = self._csv_path(robot_name)
            csv_file = file_path.open(
                'w',
                newline='',
                encoding='utf-8',
                buffering=1,
            )
            writer = csv.writer(csv_file)
            writer.writerow(self.HEADER)
            self._files[robot_name] = csv_file
            self._writers[robot_name] = writer
        return self._writers[robot_name]

    def _pose_callback(self, msg: PoseStamped, robot_name: str) -> None:
        writer = self._writer_for_robot(robot_name)
        writer.writerow(
            [
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                f'{now_wall_time():.9f}',
                robot_name,
                msg.header.frame_id,
                f'{msg.pose.position.x:.12g}',
                f'{msg.pose.position.y:.12g}',
                f'{msg.pose.position.z:.12g}',
                f'{msg.pose.orientation.x:.12g}',
                f'{msg.pose.orientation.y:.12g}',
                f'{msg.pose.orientation.z:.12g}',
                f'{msg.pose.orientation.w:.12g}',
            ]
        )
        self._counts[robot_name] += 1

        if self._counts[robot_name] % self.flush_every_n == 0:
            self._files[robot_name].flush()

        count = self._counts[robot_name]
        if count <= 5 or count % 500 == 0:
            self.file_logger.info(
                f"Recorded ground-truth pose robot='{robot_name}' "
                f"count={count} "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            )

    def close_files(self) -> None:
        for robot_name, csv_file in self._files.items():
            csv_file.flush()
            csv_file.close()
            self.file_logger.info(
                f"Closed ground-truth CSV robot='{robot_name}' "
                f"rows={self._counts.get(robot_name, 0)}"
            )
        self.file_logger.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthTrajectoryRecorder()
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
