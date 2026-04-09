#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class GroundTruthPoseExtractor(Node):
    """Extract one entity pose from TFMessage and publish it as PoseStamped."""

    def __init__(self) -> None:
        super().__init__('ground_truth_pose_extractor')

        self.declare_parameter('input_topic', 'ground_truth/pose_raw')
        self.declare_parameter('output_topic', 'ground_truth/pose')
        self.declare_parameter('target_entity_name', '')
        self.declare_parameter('output_frame_id', '')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        configured_target = self.get_parameter('target_entity_name').get_parameter_value().string_value.strip()
        configured_output_frame = self.get_parameter('output_frame_id').get_parameter_value().string_value.strip()

        self.target_entity_name = configured_target or self._default_target_name_from_namespace()
        self.output_frame_id = configured_output_frame
        self._warned_zero_stamp = False
        self._warned_empty_frame = False

        self.pose_pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.pose_sub = self.create_subscription(TFMessage, input_topic, self._pose_callback, 10)

        self.get_logger().info(
            f"Ground truth extractor ready. input='{input_topic}', output='{output_topic}', "
            f"target_entity='{self.target_entity_name}', output_frame_id='{self.output_frame_id or 'world'}'"
        )

    def _default_target_name_from_namespace(self) -> str:
        namespace = self.get_namespace().strip('/')
        if namespace:
            return f'{namespace}/robot'
        return 'robot'

    @staticmethod
    def _normalize_frame_name(name: str) -> str:
        return name.lstrip('/').strip()

    def _find_target_transform(self, msg: TFMessage) -> Optional[PoseStamped]:
        for transform in msg.transforms:
            child_name = self._normalize_frame_name(transform.child_frame_id)
            if child_name != self.target_entity_name:
                continue

            pose_msg = PoseStamped()
            pose_msg.header = transform.header
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation

            if pose_msg.header.stamp.sec == 0 and pose_msg.header.stamp.nanosec == 0:
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                if not self._warned_zero_stamp:
                    self.get_logger().warn(
                        'Incoming transform stamp is zero. Using current ROS clock for PoseStamped header stamp.'
                    )
                    self._warned_zero_stamp = True

            if not pose_msg.header.frame_id:
                pose_msg.header.frame_id = self.output_frame_id or 'world'
                if not self._warned_empty_frame:
                    self.get_logger().warn(
                        f"Incoming transform frame_id is empty. Using '{pose_msg.header.frame_id}' for PoseStamped header frame_id."
                    )
                    self._warned_empty_frame = True

            return pose_msg

        return None

    def _pose_callback(self, msg: TFMessage) -> None:
        pose_msg = self._find_target_transform(msg)
        if pose_msg is None:
            return
        self.pose_pub.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthPoseExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
