#!/usr/bin/env python3

from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from cslam_common_interfaces.msg import KeyframeOdom, PoseGraph, VizPointCloud
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


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
    """Publish one RViz marker per CSLAM keyframe cloud in the current global graph frame."""

    def __init__(self) -> None:
        super().__init__("cslam_keyframe_cloud_viewer")

        self.declare_parameter("pose_graph_topic", "/cslam/viz/pose_graph")
        self.declare_parameter("keyframe_cloud_topic", "/cslam/viz/keyframe_pointcloud")
        self.declare_parameter("keyframe_odom_topic", "/r0/cslam/keyframe_odom")
        self.declare_parameter("output_topic", "/cslam_rviz/keyframe_cloud_markers")
        self.declare_parameter("point_scale", 0.08)
        self.declare_parameter("max_points_per_keyframe", 0)

        self.pose_graph_topic = self.get_parameter("pose_graph_topic").get_parameter_value().string_value
        self.keyframe_cloud_topic = self.get_parameter("keyframe_cloud_topic").get_parameter_value().string_value
        self.keyframe_odom_topic = self.get_parameter("keyframe_odom_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.point_scale = float(self.get_parameter("point_scale").value)
        self.max_points_per_keyframe = int(self.get_parameter("max_points_per_keyframe").value)

        self.optimized_pose_cache: Dict[Key, Pose] = {}
        self.odom_pose_cache: Dict[Key, Pose] = {}
        self.cloud_cache: Dict[Key, np.ndarray] = {}
        self.last_published_keys: Set[Key] = set()
        self.global_frame_id: Optional[str] = None
        self.map_to_odom_transform: Optional[Transform] = None

        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.marker_publisher = self.create_publisher(MarkerArray, self.output_topic, marker_qos)
        self.create_subscription(PoseGraph, self.pose_graph_topic, self._pose_graph_callback, 20)
        self.create_subscription(VizPointCloud, self.keyframe_cloud_topic, self._keyframe_cloud_callback, 20)
        self.create_subscription(KeyframeOdom, self.keyframe_odom_topic, self._keyframe_odom_callback, 20)

        self.get_logger().info(
            "CSLAM keyframe cloud viewer started: "
            f"pose_graph_topic='{self.pose_graph_topic}', "
            f"keyframe_cloud_topic='{self.keyframe_cloud_topic}', "
            f"keyframe_odom_topic='{self.keyframe_odom_topic}', "
            f"output_topic='{self.output_topic}', "
            f"point_scale={self.point_scale:.3f}, "
            f"max_points_per_keyframe={self.max_points_per_keyframe}"
        )

    def _color_for_robot(self, robot_id: int) -> ColorRGBA:
        palette = (
            (0.93, 0.33, 0.31),
            (0.18, 0.54, 0.76),
            (0.20, 0.67, 0.36),
            (0.95, 0.77, 0.06),
            (0.56, 0.27, 0.68),
            (0.91, 0.49, 0.13),
        )
        r, g, b = palette[robot_id % len(palette)]
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.85)

    def _height_color(self, z: float, z_min: float, z_max: float) -> ColorRGBA:
        if z_max - z_min <= 1e-6:
            t = 0.5
        else:
            t = max(0.0, min(1.0, (z - z_min) / (z_max - z_min)))

        stops = (
            (0.00, (1.00, 0.10, 0.00)),
            (0.25, (1.00, 0.90, 0.00)),
            (0.50, (0.00, 0.90, 0.20)),
            (0.75, (0.00, 0.85, 1.00)),
            (1.00, (0.00, 0.20, 1.00)),
        )

        for idx in range(len(stops) - 1):
            left_t, left_rgb = stops[idx]
            right_t, right_rgb = stops[idx + 1]
            if t <= right_t:
                span = max(1e-6, right_t - left_t)
                local_t = (t - left_t) / span
                r = left_rgb[0] + local_t * (right_rgb[0] - left_rgb[0])
                g = left_rgb[1] + local_t * (right_rgb[1] - left_rgb[1])
                b = left_rgb[2] + local_t * (right_rgb[2] - left_rgb[2])
                return ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.95)

        r, g, b = stops[-1][1]
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.95)

    def _marker_for_key(
        self,
        key: Key,
        points_world: np.ndarray,
        point_colors: List[ColorRGBA],
        stamp,
    ) -> Marker:
        robot_id, keyframe_id = key
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.global_frame_id if self.global_frame_id else ""
        marker.ns = f"robot_{robot_id}"
        marker.id = int(keyframe_id)
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.point_scale
        marker.scale.y = self.point_scale
        marker.scale.z = 0.0
        marker.color = self._color_for_robot(robot_id)
        marker.frame_locked = False
        marker.points = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in points_world
        ]
        marker.colors = point_colors
        return marker

    def _delete_marker_for_key(self, key: Key, stamp) -> Marker:
        robot_id, keyframe_id = key
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.global_frame_id if self.global_frame_id else ""
        marker.ns = f"robot_{robot_id}"
        marker.id = int(keyframe_id)
        marker.action = Marker.DELETE
        return marker

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

    def _transform_points_to_global(
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

    def _transform_points_with_anchor(
        self,
        points_odom: np.ndarray,
        map_to_odom: Transform,
    ) -> np.ndarray:
        if len(points_odom) == 0:
            return points_odom

        translation_map_odom, quaternion_map_odom = map_to_odom
        rotation_map_odom = quaternion_to_rotation_matrix(quaternion_map_odom)
        return points_odom @ rotation_map_odom.T + translation_map_odom

    def _refresh_map_to_odom_transform(self) -> None:
        candidate_keys = sorted(set(self.optimized_pose_cache.keys()) & set(self.odom_pose_cache.keys()))
        if not candidate_keys:
            return

        anchor_key = candidate_keys[-1]
        optimized_pose = self.optimized_pose_cache[anchor_key]
        odom_pose = self.odom_pose_cache[anchor_key]
        self.map_to_odom_transform = compose_pose(optimized_pose, invert_pose(odom_pose))

    def _publish_markers(self) -> None:
        if not self.global_frame_id:
            return

        stamp = self.get_clock().now().to_msg()
        visible_keys = set(self.cloud_cache.keys()) & set(self.odom_pose_cache.keys())
        markers = []
        points_world_by_key: Dict[Key, np.ndarray] = {}

        for key in sorted(visible_keys):
            if key in self.optimized_pose_cache:
                points_world_by_key[key] = self._transform_points_to_global(
                    self.cloud_cache[key],
                    self.optimized_pose_cache[key],
                    self.odom_pose_cache[key],
                )
            elif self.map_to_odom_transform is not None:
                points_world_by_key[key] = self._transform_points_with_anchor(
                    self.cloud_cache[key],
                    self.map_to_odom_transform,
                )

        render_keys = set(points_world_by_key.keys())

        non_empty_z_arrays = [points[:, 2] for points in points_world_by_key.values() if len(points) > 0]
        if non_empty_z_arrays:
            all_z = np.concatenate(non_empty_z_arrays)
            if all_z.size > 0:
                z_min = float(np.min(all_z))
                z_max = float(np.max(all_z))
            else:
                z_min = 0.0
                z_max = 1.0
        else:
            z_min = 0.0
            z_max = 1.0

        for key in sorted(render_keys):
            points_world = points_world_by_key[key]
            point_colors = [self._height_color(float(point[2]), z_min, z_max) for point in points_world]
            markers.append(self._marker_for_key(key, points_world, point_colors, stamp))

        for key in sorted(self.last_published_keys - render_keys):
            markers.append(self._delete_marker_for_key(key, stamp))

        if not markers and not self.last_published_keys:
            return

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_publisher.publish(marker_array)
        self.last_published_keys = render_keys

    def _pose_graph_callback(self, msg: PoseGraph) -> None:
        self.global_frame_id = f"robot{msg.origin_robot_id}_map"

        for value in msg.values:
            key = (int(value.key.robot_id), int(value.key.keyframe_id))
            self.optimized_pose_cache[key] = pose_from_pose_msg(value.pose)

        self._refresh_map_to_odom_transform()
        self._publish_markers()

    def _keyframe_cloud_callback(self, msg: VizPointCloud) -> None:
        key = (int(msg.robot_id), int(msg.keyframe_id))
        self.cloud_cache[key] = self._extract_xyz_points(msg)
        self._publish_markers()

    def _keyframe_odom_callback(self, msg: KeyframeOdom) -> None:
        robot_id = self._infer_robot_id_from_odom(msg.odom)
        key = (robot_id, int(msg.id))
        self.odom_pose_cache[key] = pose_from_odom_msg(msg.odom)
        self._refresh_map_to_odom_transform()
        self._publish_markers()

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
