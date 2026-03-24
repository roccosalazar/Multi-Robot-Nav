#!/usr/bin/env python3

from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfPrefixRelay(Node):
    def __init__(self) -> None:
        """
        Relay TF from robot-scoped topics to global topics, adding a fixed robot prefix
        to ALL frame IDs while preserving TF tree links.

        Parameters:
        - robot_ns: fixed prefix to prepend
        """
        super().__init__("tf_prefix_relay")

        self.declare_parameter("robot_ns", "")
        robot_ns = self.get_parameter("robot_ns").get_parameter_value().string_value.strip("/")

        self.robot_prefix: str = robot_ns
        if not self.robot_prefix:
            raise ValueError("Parameter 'robot_ns' must be a non-empty string")

        self.prefix_with_sep: str = f"{self.robot_prefix}/"

        # Cache for static TFs: key=(parent, child), value=latest transform
        self.static_cache: Dict[Tuple[str, str], TransformStamped] = {}

        tf_in_topic: str = f"/{self.robot_prefix}/tf"
        tf_static_in_topic: str = f"/{self.robot_prefix}/tf_static"

        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_subscription = self.create_subscription(
            TFMessage, tf_in_topic, self.tf_callback, tf_qos
        )
        self.tf_static_subscription = self.create_subscription(
            TFMessage, tf_static_in_topic, self.tf_static_callback, tf_static_qos
        )

        self.tf_publisher = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.tf_static_publisher = self.create_publisher(TFMessage, "/tf_static", tf_static_qos)

        self.get_logger().info(
            f"TF prefix relay started with robot_prefix='{self.robot_prefix}', "
            f"subscribing to '{tf_in_topic}' and '{tf_static_in_topic}', "
            f"publishing to '/tf' and '/tf_static'"
        )

    def add_prefix(self, frame_id: str) -> str:
        """
        Prepend '<robot_prefix>/' to frame_id if not already present.
        Only frame id string changes; TF links stay identical.
        """
        clean_frame_id: str = frame_id.strip().lstrip("/")
        if not clean_frame_id:
            return clean_frame_id

        if clean_frame_id.startswith(self.prefix_with_sep):
            return clean_frame_id

        return f"{self.prefix_with_sep}{clean_frame_id}"

    def rewrite_transforms(self, transforms: List[TransformStamped]) -> List[TransformStamped]:
        """
        Rewrite parent and child frame names for all transforms.
        """
        output_transforms: List[TransformStamped] = []

        for transform in transforms:
            new_transform = TransformStamped()

            # copy header stamp and rewrite frame id
            new_transform.header.stamp = transform.header.stamp
            new_transform.header.frame_id = self.add_prefix(transform.header.frame_id)

            # rewrite child frame id
            new_transform.child_frame_id = self.add_prefix(transform.child_frame_id)

            # copy transform payload
            new_transform.transform = transform.transform

            output_transforms.append(new_transform)

        return output_transforms

    def tf_callback(self, msg: TFMessage) -> None:
        rewritten = self.rewrite_transforms(msg.transforms)
        if rewritten:
            self.tf_publisher.publish(TFMessage(transforms=rewritten))

    def tf_static_callback(self, msg: TFMessage) -> None:
        rewritten = self.rewrite_transforms(msg.transforms)
        if not rewritten:
            return

        # Update cache and republish full static tree
        for t in rewritten:
            key = (t.header.frame_id, t.child_frame_id)
            self.static_cache[key] = t

        self.tf_static_publisher.publish(TFMessage(transforms=list(self.static_cache.values())))


def main() -> None:
    rclpy.init()
    node: TfPrefixRelay | None = None

    try:
        node = TfPrefixRelay()
        rclpy.spin(node)
    except ValueError as exc:
        if rclpy.ok():
            temp_node = Node("tf_prefix_relay_error")
            temp_node.get_logger().error(str(exc))
            temp_node.destroy_node()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()