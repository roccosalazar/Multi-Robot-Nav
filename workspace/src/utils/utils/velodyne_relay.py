#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException


class VelodyneRelay(Node):
    def __init__(self) -> None:
        super().__init__("velodyne_relay")

        self.declare_parameter("robot_ns", "")
        robot_ns = self.get_parameter("robot_ns").get_parameter_value().string_value.strip("/")
        if not robot_ns:
            raise ValueError("Parameter 'robot_ns' must be a non-empty string")

        self.robot_ns = robot_ns
        self.base_frame = f"{self.robot_ns}/base_link"
        self.velodyne_frame = f"{self.robot_ns}/velodyne"

        self.input_topic = f"/{self.robot_ns}/sensors/lidar3d_0/points"
        self.output_topic = f"/{self.robot_ns}/velodyne_points"

        # frame lidar reale
        self.lidar_frame_candidates: List[str] = [
            f"{self.robot_ns}/lidar3d_0_laser",
        ]

        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_cb,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_sent = False
        self.timer = self.create_timer(1.0, self.try_publish_static_tf)

        self.get_logger().info(
            f"Velodyne relay started for robot='{self.robot_ns}' "
            f"({self.input_topic} -> {self.output_topic}, frame={self.velodyne_frame})"
        )

    def cloud_cb(self, msg: PointCloud2) -> None:
        out = PointCloud2()

        # Header
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.velodyne_frame

        # Metadata
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense

        # Payload
        out.data = msg.data

        self.pub.publish(out)

    def try_publish_static_tf(self) -> None:
        if self.static_sent:
            return

        for lidar_frame in self.lidar_frame_candidates:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,          # target frame
                    lidar_frame,              # source frame
                    rclpy.time.Time(),        # latest
                    timeout=Duration(seconds=0.2),
                )

                static_tf = TransformStamped()
                static_tf.header.stamp = self.get_clock().now().to_msg()
                static_tf.header.frame_id = self.base_frame
                static_tf.child_frame_id = self.velodyne_frame
                static_tf.transform = tf.transform

                self.static_broadcaster.sendTransform(static_tf)
                self.static_sent = True
                self.get_logger().info(
                    f"Published static TF: {self.base_frame} -> {self.velodyne_frame} "
                    f"(copied from {self.base_frame} -> {lidar_frame})"
                )
                return

            except TransformException:
                continue

        self.get_logger().warn(
            f"Cannot resolve lidar TF yet. Tried: {self.lidar_frame_candidates}",
            throttle_duration_sec=5.0,
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = VelodyneRelay()
        rclpy.spin(node)
    except ValueError as exc:
        if rclpy.ok():
            temp = Node("velodyne_relay_error")
            temp.get_logger().error(str(exc))
            temp.destroy_node()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()