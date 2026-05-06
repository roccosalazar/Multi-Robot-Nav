#!/usr/bin/env python3

import math
from copy import deepcopy
from typing import Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def quat_inverse(q: Quaternion) -> Quaternion:
    norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    if norm_sq <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (-q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, q[3] / norm_sq)


def quat_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    q_v = (v[0], v[1], v[2], 0.0)
    q_rot = quat_multiply(quat_multiply(q, q_v), quat_inverse(q))
    return (q_rot[0], q_rot[1], q_rot[2])


def compose_pose(
    translation_a: Vector3,
    rotation_a: Quaternion,
    translation_b: Vector3,
    rotation_b: Quaternion,
) -> Tuple[Vector3, Quaternion]:
    rotated_b = rotate_vector(rotation_a, translation_b)
    translation = (
        translation_a[0] + rotated_b[0],
        translation_a[1] + rotated_b[1],
        translation_a[2] + rotated_b[2],
    )
    rotation = quat_normalize(quat_multiply(rotation_a, rotation_b))
    return translation, rotation


def pose_from_odom(msg: Odometry) -> Tuple[Vector3, Quaternion]:
    pose = msg.pose.pose
    return (
        (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        quat_normalize(
            (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        ),
    )


def transform_to_pose(msg: TransformStamped) -> Tuple[Vector3, Quaternion]:
    transform = msg.transform
    return (
        (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ),
        quat_normalize(
            (
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            )
        ),
    )


class FastlioOdomAdapter(Node):
    """Republish FAST_LIO odometry with robot-scoped odom/base frames."""

    def __init__(self) -> None:
        super().__init__("fastlio_odom_adapter")

        self.declare_parameter("input_odom_topic", "fastlio/raw_odom")
        self.declare_parameter("output_odom_topic", "fastlio/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("fastlio_body_frame_id", "")
        self.declare_parameter("use_body_to_base_tf", True)
        self.declare_parameter("lookup_timeout_sec", 0.05)
        self.declare_parameter("broadcast_tf", True)

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.fastlio_body_frame_id = self.get_parameter("fastlio_body_frame_id").value
        self.use_body_to_base_tf = bool(self.get_parameter("use_body_to_base_tf").value)
        self.lookup_timeout_sec = float(self.get_parameter("lookup_timeout_sec").value)
        self.broadcast_tf = bool(self.get_parameter("broadcast_tf").value)

        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None
        if self.use_body_to_base_tf and self.fastlio_body_frame_id:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.create_subscription(Odometry, self.input_odom_topic, self._odom_callback, 20)

        self._warned_tf_lookup = False
        self.get_logger().info(
            "FAST_LIO odom adapter started: "
            f"input='{self.input_odom_topic}', output='{self.output_odom_topic}', "
            f"odom_frame='{self.odom_frame_id}', base_frame='{self.base_frame_id}', "
            f"fastlio_body_frame='{self.fastlio_body_frame_id}', broadcast_tf={self.broadcast_tf}"
        )

    def _lookup_body_to_base(self, stamp) -> Tuple[Vector3, Quaternion]:
        if not self.tf_buffer or not self.fastlio_body_frame_id:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.fastlio_body_frame_id,
                self.base_frame_id,
                Time.from_msg(stamp, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
            self._warned_tf_lookup = False
            return transform_to_pose(transform)
        except TransformException as exc:
            if not self._warned_tf_lookup:
                self.get_logger().warn(
                    "Could not lookup FAST_LIO body->base transform "
                    f"'{self.fastlio_body_frame_id}' <- '{self.base_frame_id}': {exc}. "
                    "Using identity until TF is available."
                )
                self._warned_tf_lookup = True
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

    def _odom_callback(self, msg: Odometry) -> None:
        odom_to_body_translation, odom_to_body_rotation = pose_from_odom(msg)
        body_to_base_translation, body_to_base_rotation = self._lookup_body_to_base(msg.header.stamp)
        odom_to_base_translation, odom_to_base_rotation = compose_pose(
            odom_to_body_translation,
            odom_to_body_rotation,
            body_to_base_translation,
            body_to_base_rotation,
        )

        adapted = Odometry()
        adapted.header = deepcopy(msg.header)
        adapted.header.frame_id = self.odom_frame_id
        adapted.child_frame_id = self.base_frame_id
        adapted.pose = deepcopy(msg.pose)
        adapted.pose.pose.position.x = odom_to_base_translation[0]
        adapted.pose.pose.position.y = odom_to_base_translation[1]
        adapted.pose.pose.position.z = odom_to_base_translation[2]
        adapted.pose.pose.orientation.x = odom_to_base_rotation[0]
        adapted.pose.pose.orientation.y = odom_to_base_rotation[1]
        adapted.pose.pose.orientation.z = odom_to_base_rotation[2]
        adapted.pose.pose.orientation.w = odom_to_base_rotation[3]
        adapted.twist = deepcopy(msg.twist)

        self.odom_publisher.publish(adapted)

        if self.broadcast_tf:
            self._publish_tf(adapted)

    def _publish_tf(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = FastlioOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
