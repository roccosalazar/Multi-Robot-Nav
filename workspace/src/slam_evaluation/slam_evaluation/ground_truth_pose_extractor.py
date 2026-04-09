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

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        configured_target = self.get_parameter('target_entity_name').get_parameter_value().string_value.strip()

        self.target_entity_name = configured_target or self._default_target_name_from_namespace()

        self.pose_pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.pose_sub = self.create_subscription(TFMessage, input_topic, self._pose_callback, 10)

        self.get_logger().info(
            f"Ground truth extractor ready. input='{input_topic}', output='{output_topic}', "
            f"target_entity='{self.target_entity_name}'"
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
