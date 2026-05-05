#!/usr/bin/env python3

from typing import Dict, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from cslam_common_interfaces.msg import KeyframeOdom, PoseGraph, VizPointCloud
from geometry_msgs.msg import Pose as PoseMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


Key = Tuple[int, int]
Pose = Tuple[np.ndarray, np.ndarray]
Transform = Tuple[np.ndarray, np.ndarray]


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return quaternion / norm


def quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    return np.array(
        [
            -float(quaternion[0]),
            -float(quaternion[1]),
            -float(quaternion[2]),
            float(quaternion[3]),
        ],
        dtype=np.float32,
    )


def quat_inverse(quaternion: np.ndarray) -> np.ndarray:
    norm_sq = float(np.dot(quaternion, quaternion))
    if norm_sq <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return quat_conjugate(quaternion) / norm_sq


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = [float(v) for v in q1]
    x2, y2, z2, w2 = [float(v) for v in q2]
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float32,
    )


def rotate_vector(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    q_vector = np.array([float(vector[0]), float(vector[1]), float(vector[2]), 0.0], dtype=np.float32)
    rotated = quat_multiply(quat_multiply(quaternion, q_vector), quat_inverse(quaternion))
    return rotated[:3]


def invert_pose(pose: Pose) -> Pose:
    translation, quaternion = pose
    quaternion_inv = normalize_quaternion(quat_inverse(quaternion))
    translation_inv = rotate_vector(quaternion_inv, -translation)
    return translation_inv.astype(np.float32), quaternion_inv.astype(np.float32)


def compose_pose(pose_a: Pose, pose_b: Pose) -> Pose:
    translation_a, quaternion_a = pose_a
    translation_b, quaternion_b = pose_b
    translation = translation_a + rotate_vector(quaternion_a, translation_b)
    quaternion = normalize_quaternion(quat_multiply(quaternion_a, quaternion_b))
    return translation.astype(np.float32), quaternion.astype(np.float32)


def pose_from_pose_msg(msg: PoseMsg) -> Pose:
    translation = np.array(
        [float(msg.position.x), float(msg.position.y), float(msg.position.z)],
        dtype=np.float32,
    )
    quaternion = normalize_quaternion(
        np.array(
            [
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ],
            dtype=np.float32,
        )
    )
    return translation, quaternion


def pose_from_odom_msg(msg: Odometry) -> Pose:
    return pose_from_pose_msg(msg.pose.pose)


class CslamKeyframeCloudViewer(Node):
    """Build a single global PointCloud2 from CSLAM keyframe clouds."""

    def __init__(self) -> None:
        super().__init__("cslam_keyframe_cloud_viewer")

        self.declare_parameter("pose_graph_topic", "/cslam/viz/pose_graph")
        self.declare_parameter("keyframe_cloud_topic", "/cslam/viz/keyframe_pointcloud")
        self.declare_parameter("keyframe_odom_topic", "/r0/cslam/keyframe_odom")
        self.declare_parameter("output_topic", "/cslam_rviz/map_points")
        self.declare_parameter("point_scale", 0.001)
        self.declare_parameter("max_points_per_keyframe", 1000)
        self.declare_parameter("keyframe_stride", 10)
        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("update_only_when_subscribed", True)

        self.pose_graph_topic = self.get_parameter("pose_graph_topic").get_parameter_value().string_value
        self.keyframe_cloud_topic = self.get_parameter("keyframe_cloud_topic").get_parameter_value().string_value
        self.keyframe_odom_topic = self.get_parameter("keyframe_odom_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.max_points_per_keyframe = int(self.get_parameter("max_points_per_keyframe").value)
        self.keyframe_stride = max(1, int(self.get_parameter("keyframe_stride").value))
        self.voxel_size = max(0.0, float(self.get_parameter("voxel_size").value))
        self.publish_period_sec = max(0.05, float(self.get_parameter("publish_period_sec").value))
        self.update_only_when_subscribed = bool(self.get_parameter("update_only_when_subscribed").value)

        self.optimized_pose_cache: Dict[Key, Pose] = {}
        self.odom_pose_cache: Dict[Key, Pose] = {}
        self.odom_frame_cache: Dict[Key, Tuple[str, str]] = {}
        self.cloud_cache: Dict[Key, np.ndarray] = {}
        self.cloud_frame_cache: Dict[Key, str] = {}
        self.global_frame_id: Optional[str] = None
        self.map_to_odom_transform: Optional[Transform] = None
        self.last_cloud_stamp = None
        self.map_dirty = False
        self.publish_counter = 0

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cloud_publisher = self.create_publisher(PointCloud2, self.output_topic, cloud_qos)
        self.create_subscription(PoseGraph, self.pose_graph_topic, self._pose_graph_callback, 20)
        self.create_subscription(VizPointCloud, self.keyframe_cloud_topic, self._keyframe_cloud_callback, 20)
        self.create_subscription(KeyframeOdom, self.keyframe_odom_topic, self._keyframe_odom_callback, 20)
        self.create_timer(self.publish_period_sec, self._publish_map_if_needed)

        self.get_logger().info(
            "CSLAM global map viewer started: "
            f"pose_graph_topic='{self.pose_graph_topic}', "
            f"keyframe_cloud_topic='{self.keyframe_cloud_topic}', "
            f"keyframe_odom_topic='{self.keyframe_odom_topic}', "
            f"output_topic='{self.output_topic}', "
            f"voxel_size={self.voxel_size:.3f}, "
            f"max_points_per_keyframe={self.max_points_per_keyframe}, "
            f"keyframe_stride={self.keyframe_stride}, "
            f"publish_period_sec={self.publish_period_sec:.2f}"
        )

    def _extract_xyz_points(self, msg: VizPointCloud) -> np.ndarray:
        cloud = msg.pointcloud
        if cloud.width * cloud.height == 0 or cloud.point_step < 12:
            return np.empty((0, 3), dtype=np.float32)

        dtype = np.dtype(
            {
                "names": ("x", "y", "z"),
                "formats": ("<f4", "<f4", "<f4"),
                "offsets": (0, 4, 8),
                "itemsize": int(cloud.point_step),
            }
        )
        raw = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width * cloud.height)
        points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)
        finite_mask = np.isfinite(points).all(axis=1)
        points = points[finite_mask]

        if self.max_points_per_keyframe > 0 and len(points) > self.max_points_per_keyframe:
            stride = max(1, len(points) // self.max_points_per_keyframe)
            points = points[::stride][: self.max_points_per_keyframe]

        return points

    def _transform_odom_points_to_global(
        self,
        points_odom: np.ndarray,
        optimized_pose: Pose,
        odom_pose: Pose,
    ) -> np.ndarray:
        if len(points_odom) == 0:
            return points_odom

        translation_map, quaternion_map = optimized_pose
        translation_odom, quaternion_odom = odom_pose

        rotation_map = quaternion_to_rotation_matrix(quaternion_map)
        rotation_odom = quaternion_to_rotation_matrix(quaternion_odom)

        points_base = (points_odom - translation_odom) @ rotation_odom
        return points_base @ rotation_map.T + translation_map

    def _transform_local_points_to_global(
        self,
        points_local: np.ndarray,
        optimized_pose: Pose,
    ) -> np.ndarray:
        if len(points_local) == 0:
            return points_local

        translation_map, quaternion_map = optimized_pose
        rotation_map = quaternion_to_rotation_matrix(quaternion_map)
        return points_local @ rotation_map.T + translation_map

    def _transform_odom_points_with_anchor(
        self,
        points_odom: np.ndarray,
        map_to_odom: Transform,
    ) -> np.ndarray:
        if len(points_odom) == 0:
            return points_odom

        translation_map_odom, quaternion_map_odom = map_to_odom
        rotation_map_odom = quaternion_to_rotation_matrix(quaternion_map_odom)
        return points_odom @ rotation_map_odom.T + translation_map_odom

    def _transform_local_points_with_anchor(
        self,
        points_local: np.ndarray,
        odom_pose: Pose,
        map_to_odom: Transform,
    ) -> np.ndarray:
        if len(points_local) == 0:
            return points_local

        translation_odom, quaternion_odom = odom_pose
        rotation_odom = quaternion_to_rotation_matrix(quaternion_odom)
        points_odom = points_local @ rotation_odom.T + translation_odom
        return self._transform_odom_points_with_anchor(points_odom, map_to_odom)

    def _cloud_is_in_odom_frame(self, key: Key) -> bool:
        cloud_frame = self.cloud_frame_cache.get(key, "").strip("/")
        odom_frames = self.odom_frame_cache.get(key)
        if not odom_frames:
            return True

        odom_frame, base_frame = [frame.strip("/") for frame in odom_frames]
        if not cloud_frame:
            return True
        if cloud_frame == odom_frame:
            return True
        if cloud_frame == base_frame:
            return False
        return True

    def _refresh_map_to_odom_transform(self) -> None:
        candidate_keys = sorted(set(self.optimized_pose_cache.keys()) & set(self.odom_pose_cache.keys()))
        if not candidate_keys:
            return

        anchor_key = candidate_keys[-1]
        optimized_pose = self.optimized_pose_cache[anchor_key]
        odom_pose = self.odom_pose_cache[anchor_key]
        self.map_to_odom_transform = compose_pose(optimized_pose, invert_pose(odom_pose))

    def _mark_dirty(self) -> None:
        self.map_dirty = True

    def _build_world_cloud(self) -> np.ndarray:
        visible_keys = sorted(set(self.cloud_cache.keys()) & set(self.odom_pose_cache.keys()))
        world_chunks = []

        for key in visible_keys:
            points_local = self.cloud_cache[key]
            if key in self.optimized_pose_cache:
                if self._cloud_is_in_odom_frame(key):
                    points_world = self._transform_odom_points_to_global(
                        points_local,
                        self.optimized_pose_cache[key],
                        self.odom_pose_cache[key],
                    )
                else:
                    points_world = self._transform_local_points_to_global(
                        points_local,
                        self.optimized_pose_cache[key],
                    )
            elif self.map_to_odom_transform is not None:
                if self._cloud_is_in_odom_frame(key):
                    points_world = self._transform_odom_points_with_anchor(
                        points_local,
                        self.map_to_odom_transform,
                    )
                else:
                    points_world = self._transform_local_points_with_anchor(
                        points_local,
                        self.odom_pose_cache[key],
                        self.map_to_odom_transform,
                    )
            else:
                continue

            if len(points_world) > 0:
                world_chunks.append(points_world.astype(np.float32, copy=False))

        if not world_chunks:
            return np.empty((0, 3), dtype=np.float32)

        world_cloud = np.concatenate(world_chunks, axis=0)
        return self._voxel_downsample(world_cloud)

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0 or self.voxel_size <= 0.0:
            return points

        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        unique_indices.sort()
        return points[unique_indices]

    def _make_pointcloud2_msg(self, points: np.ndarray) -> PointCloud2:
        header = Header()
        header.frame_id = self.global_frame_id if self.global_frame_id else ""
        header.stamp = self.last_cloud_stamp if self.last_cloud_stamp is not None else self.get_clock().now().to_msg()

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(len(points))
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = np.asarray(points, dtype=np.float32).tobytes()
        return msg

    def _publish_map_if_needed(self) -> None:
        if not self.map_dirty or not self.global_frame_id:
            return

        if self.update_only_when_subscribed and self.cloud_publisher.get_subscription_count() == 0:
            return

        world_cloud = self._build_world_cloud()
        cloud_msg = self._make_pointcloud2_msg(world_cloud)
        self.cloud_publisher.publish(cloud_msg)
        self.map_dirty = False
        self.publish_counter += 1

        if self.publish_counter <= 3 or self.publish_counter % 20 == 0:
            self.get_logger().info(
                "Published CSLAM global map "
                f"keyframes={len(self.cloud_cache)} points={len(world_cloud)} "
                f"optimized_poses={len(self.optimized_pose_cache)}"
            )

    def _pose_graph_callback(self, msg: PoseGraph) -> None:
        self.global_frame_id = f"robot{msg.origin_robot_id}_map"
        current_keys: Set[Key] = set()

        for value in msg.values:
            key = (int(value.key.robot_id), int(value.key.keyframe_id))
            self.optimized_pose_cache[key] = pose_from_pose_msg(value.pose)
            current_keys.add(key)

        stale_keys = set(self.optimized_pose_cache.keys()) - current_keys
        for key in stale_keys:
            self.optimized_pose_cache.pop(key, None)

        self._refresh_map_to_odom_transform()
        self.last_cloud_stamp = self.get_clock().now().to_msg()
        self._mark_dirty()

    def _keyframe_cloud_callback(self, msg: VizPointCloud) -> None:
        key = (int(msg.robot_id), int(msg.keyframe_id))
        if self.keyframe_stride > 1 and key[1] % self.keyframe_stride != 0:
            return

        self.cloud_cache[key] = self._extract_xyz_points(msg)
        self.cloud_frame_cache[key] = msg.pointcloud.header.frame_id
        self.last_cloud_stamp = msg.pointcloud.header.stamp
        self._mark_dirty()

    def _keyframe_odom_callback(self, msg: KeyframeOdom) -> None:
        robot_id = self._infer_robot_id_from_odom(msg.odom)
        key = (robot_id, int(msg.id))
        self.odom_pose_cache[key] = pose_from_odom_msg(msg.odom)
        self.odom_frame_cache[key] = (msg.odom.header.frame_id, msg.odom.child_frame_id)
        self.last_cloud_stamp = msg.odom.header.stamp
        self._refresh_map_to_odom_transform()
        self._mark_dirty()

    def _infer_robot_id_from_odom(self, odom: Odometry) -> int:
        frame_tokens = [
            odom.header.frame_id.strip("/"),
            odom.child_frame_id.strip("/"),
        ]
        for token in frame_tokens:
            if token.startswith("r") and "/" in token:
                prefix = token.split("/", 1)[0]
                suffix = prefix[1:]
                if suffix.isdigit():
                    return int(suffix)

        self.get_logger().warn(
            "Could not infer robot_id from KeyframeOdom frames "
            f"header.frame_id='{odom.header.frame_id}' child_frame_id='{odom.child_frame_id}'. "
            "Falling back to robot_id=0."
        )
        return 0


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = CslamKeyframeCloudViewer()
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
