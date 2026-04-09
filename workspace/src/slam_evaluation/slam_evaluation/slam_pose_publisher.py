#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SlamPosePublisher(Node):
    """Publish SLAM estimated pose from TF as PoseStamped."""

    def __init__(self) -> None:
        super().__init__('slam_pose_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_topic', 'slam/pose')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('lookup_timeout_sec', 0.1)
        self.declare_parameter('position_offset_x', 0.0)
        self.declare_parameter('position_offset_y', 0.0)
        self.declare_parameter('position_offset_z', 0.0)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value.strip('/') or 'map'
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value.strip('/') or 'base_link'
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.lookup_timeout_sec = float(self.get_parameter('lookup_timeout_sec').value)
        self.position_offset_x = float(self.get_parameter('position_offset_x').value)
        self.position_offset_y = float(self.get_parameter('position_offset_y').value)
        self.position_offset_z = float(self.get_parameter('position_offset_z').value)

        self.target_frame = self._resolve_target_frame()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1e-3), self._publish_pose)

        self._warned_lookup = False

        self.get_logger().info(
            f"SlamPosePublisher started: tf '{self.map_frame}' -> '{self.target_frame}', output '{self.output_topic}', "
            f"offset=({self.position_offset_x:.3f}, {self.position_offset_y:.3f}, {self.position_offset_z:.3f})"
        )

    def _resolve_target_frame(self) -> str:
        if '/' in self.base_frame:
            return self.base_frame

        namespace = self.get_namespace().strip('/')
        if namespace:
            return f'{namespace}/{self.base_frame}'

        return self.base_frame

    def _publish_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
        except TransformException as exc:
            if not self._warned_lookup:
                self.get_logger().warn(
                    f"Cannot lookup TF '{self.map_frame}' -> '{self.target_frame}' yet: {exc}"
                )
                self._warned_lookup = True
            return

        self._warned_lookup = False

        pose_msg = PoseStamped()
        pose_msg.header = transform.header
        if pose_msg.header.stamp.sec == 0 and pose_msg.header.stamp.nanosec == 0:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
        if not pose_msg.header.frame_id:
            pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.position.x = transform.transform.translation.x + self.position_offset_x
        pose_msg.pose.position.y = transform.transform.translation.y + self.position_offset_y
        pose_msg.pose.position.z = transform.transform.translation.z + self.position_offset_z
        pose_msg.pose.orientation = transform.transform.rotation

        self.pose_pub.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
