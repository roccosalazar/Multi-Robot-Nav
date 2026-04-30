#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, Optional, Sequence, Tuple

import rclpy
from cslam_common_interfaces.msg import OptimizerState
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def quat_conjugate(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def quat_inverse(q: Quaternion) -> Quaternion:
    norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    if norm_sq <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    conj = quat_conjugate(q)
    return (conj[0] / norm_sq, conj[1] / norm_sq, conj[2] / norm_sq, conj[3] / norm_sq)


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


def invert_pose(translation: Vector3, rotation: Quaternion) -> Tuple[Vector3, Quaternion]:
    rotation_inv = quat_normalize(quat_inverse(rotation))
    translation_inv = rotate_vector(rotation_inv, (-translation[0], -translation[1], -translation[2]))
    return translation_inv, rotation_inv


def pose_from_pose_msg(pose) -> Tuple[Vector3, Quaternion]:
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


class CslamOdomTfBridge(Node):
    """Bridge CSLAM global pose and local odometry into a coherent map->odom TF."""

    def __init__(self) -> None:
        super().__init__("cslam_odom_tf_bridge")

        self.declare_parameter("cslam_pose_topic", "cslam/current_pose_estimate")
        self.declare_parameter("optimizer_state_topic", "cslam/optimizer_state")
        self.declare_parameter("odom_topic", "scan_matching_odometry/odom")
        self.declare_parameter("max_anchor_time_diff_sec", 2.0)
        self.declare_parameter("odom_buffer_size", 200)
        self.declare_parameter("publish_on_odom", True)
        self.declare_parameter("initialize_anchor_from_first_pose", True)

        self.cslam_pose_topic = self.get_parameter("cslam_pose_topic").get_parameter_value().string_value
        self.optimizer_state_topic = (
            self.get_parameter("optimizer_state_topic").get_parameter_value().string_value
        )
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.max_anchor_time_diff = float(self.get_parameter("max_anchor_time_diff_sec").value)
        self.odom_buffer_size = int(self.get_parameter("odom_buffer_size").value)
        self.publish_on_odom = bool(self.get_parameter("publish_on_odom").value)
        self.initialize_anchor_from_first_pose = bool(
            self.get_parameter("initialize_anchor_from_first_pose").value
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_buffer: Deque[Odometry] = deque(maxlen=max(1, self.odom_buffer_size))
        self.map_to_odom_translation: Optional[Vector3] = None
        self.map_to_odom_rotation: Optional[Quaternion] = None
        self.global_frame_id: Optional[str] = None
        self.odom_frame_id: Optional[str] = None
        self.base_frame_id: Optional[str] = None
        self.anchor_count = 0
        self.pending_anchor_update = False
        self.previous_optimizer_state: Optional[int] = None
        self._warned_missing_odom = False
        self._warned_frame_mismatch = False
        self._warned_time_diff = False

        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)
        self.create_subscription(PoseStamped, self.cslam_pose_topic, self._cslam_pose_callback, 20)
        self.create_subscription(
            OptimizerState,
            self.optimizer_state_topic,
            self._optimizer_state_callback,
            20,
        )

        self.get_logger().info(
            "CSLAM TF bridge started: "
            f"cslam_pose_topic='{self.cslam_pose_topic}', "
            f"optimizer_state_topic='{self.optimizer_state_topic}', "
            f"odom_topic='{self.odom_topic}', "
            f"max_anchor_time_diff_sec={self.max_anchor_time_diff:.3f}, "
            f"odom_buffer_size={self.odom_buffer_size}, "
            f"initialize_anchor_from_first_pose={self.initialize_anchor_from_first_pose}"
        )

    def _stamp_to_time(self, stamp) -> Time:
        return Time.from_msg(stamp, clock_type=self.get_clock().clock_type)

    def _stamp_diff_sec(self, stamp_a, stamp_b) -> float:
        return abs((self._stamp_to_time(stamp_a) - self._stamp_to_time(stamp_b)).nanoseconds) * 1e-9

    def _find_nearest_odom(self, pose_msg: PoseStamped) -> Tuple[Optional[Odometry], Optional[float]]:
        if not self.odom_buffer:
            return None, None

        best_msg: Optional[Odometry] = None
        best_dt: Optional[float] = None
        for odom_msg in self.odom_buffer:
            dt = self._stamp_diff_sec(pose_msg.header.stamp, odom_msg.header.stamp)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_msg = odom_msg

        return best_msg, best_dt

    def _compute_anchor_from_pose(self, pose_msg: PoseStamped) -> bool:
        odom_msg, dt = self._find_nearest_odom(pose_msg)
        if odom_msg is None or dt is None:
            if not self._warned_missing_odom:
                self.get_logger().warn(
                    "Cannot update map->odom anchor yet because no odometry samples are buffered."
                )
                self._warned_missing_odom = True
            return False

        self._warned_missing_odom = False

        if dt > self.max_anchor_time_diff:
            if not self._warned_time_diff:
                self.get_logger().warn(
                    "Skipping CSLAM anchor update because nearest pose/odom timestamps differ too much: "
                    f"{dt:.3f}s > {self.max_anchor_time_diff:.3f}s"
                )
                self._warned_time_diff = True
            return False
        self._warned_time_diff = False

        global_translation, global_rotation = pose_from_pose_msg(pose_msg.pose)
        odom_translation, odom_rotation = pose_from_pose_msg(odom_msg.pose.pose)
        odom_inv_translation, odom_inv_rotation = invert_pose(odom_translation, odom_rotation)
        map_to_odom_translation, map_to_odom_rotation = compose_pose(
            global_translation,
            global_rotation,
            odom_inv_translation,
            odom_inv_rotation,
        )

        self.map_to_odom_translation = map_to_odom_translation
        self.map_to_odom_rotation = map_to_odom_rotation
        self.global_frame_id = pose_msg.header.frame_id

        if self.odom_frame_id is not None and self.odom_frame_id != odom_msg.header.frame_id:
            self._warned_frame_mismatch = False

        if self.odom_frame_id is None:
            self.odom_frame_id = odom_msg.header.frame_id
        elif self.odom_frame_id != odom_msg.header.frame_id and not self._warned_frame_mismatch:
            self.get_logger().warn(
                "Odom frame changed during runtime: "
                f"'{self.odom_frame_id}' -> '{odom_msg.header.frame_id}'. Using latest value."
            )
            self.odom_frame_id = odom_msg.header.frame_id
            self._warned_frame_mismatch = True

        self.base_frame_id = odom_msg.child_frame_id
        self.anchor_count += 1
        self.pending_anchor_update = False

        self.get_logger().info(
            "Updated map->odom anchor after optimization "
            f"global_frame='{self.global_frame_id}' odom_frame='{self.odom_frame_id}' "
            f"base_frame='{odom_msg.child_frame_id}' dt={dt:.3f}s "
            f"translation=({map_to_odom_translation[0]:.3f}, {map_to_odom_translation[1]:.3f}, {map_to_odom_translation[2]:.3f})"
        )

        self._publish_map_to_odom(odom_msg.header.stamp)
        return True

    def _publish_map_to_odom(self, stamp) -> None:
        if self.map_to_odom_translation is None or self.map_to_odom_rotation is None:
            return
        if self.global_frame_id is None or self.odom_frame_id is None:
            return

        msg = TransformStamped()
        msg.header.stamp = stamp
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame_id
        msg.child_frame_id = self.odom_frame_id
        msg.transform.translation.x = self.map_to_odom_translation[0]
        msg.transform.translation.y = self.map_to_odom_translation[1]
        msg.transform.translation.z = self.map_to_odom_translation[2]
        msg.transform.rotation.x = self.map_to_odom_rotation[0]
        msg.transform.rotation.y = self.map_to_odom_rotation[1]
        msg.transform.rotation.z = self.map_to_odom_rotation[2]
        msg.transform.rotation.w = self.map_to_odom_rotation[3]
        self.tf_broadcaster.sendTransform(msg)

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom_buffer.append(msg)
        self.odom_frame_id = msg.header.frame_id
        self.base_frame_id = msg.child_frame_id

        if self.publish_on_odom:
            self._publish_map_to_odom(msg.header.stamp)

    def _cslam_pose_callback(self, msg: PoseStamped) -> None:
        if self.pending_anchor_update:
            self._compute_anchor_from_pose(msg)
            return

        if (
            self.initialize_anchor_from_first_pose
            and self.map_to_odom_translation is None
            and self.map_to_odom_rotation is None
        ):
            if self._compute_anchor_from_pose(msg):
                self.get_logger().info(
                    "Initialized map->odom anchor from first CSLAM pose sample."
                )

    def _optimizer_state_callback(self, msg: OptimizerState) -> None:
        previous_state = self.previous_optimizer_state
        self.previous_optimizer_state = int(msg.state)

        if previous_state is None:
            return

        optimization_finished = (
            previous_state == OptimizerState.OPTIMIZATION and msg.state == OptimizerState.IDLE
        )
        if not optimization_finished:
            return

        self.pending_anchor_update = True
        self.get_logger().info(
            "Optimization finished. Waiting for the next CSLAM pose sample to refresh map->odom."
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = CslamOdomTfBridge()
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
