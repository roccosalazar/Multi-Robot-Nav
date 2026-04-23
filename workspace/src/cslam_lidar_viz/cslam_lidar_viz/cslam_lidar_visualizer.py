from collections import Counter
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from cslam.utils import point_cloud2 as pc2_utils
from cslam_common_interfaces.msg import PoseGraph, PoseGraphEdge, VizPointCloud


Key = Tuple[int, int]


@dataclass
class CachedPoseGraph:
    robot_id: int
    origin_robot_id: int
    values: List
    edges: List[PoseGraphEdge]


def pose_msg_to_matrix(pose_msg) -> np.ndarray:
    qx = pose_msg.orientation.x
    qy = pose_msg.orientation.y
    qz = pose_msg.orientation.z
    qw = pose_msg.orientation.w

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rotation = np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)

    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation
    transform[0, 3] = pose_msg.position.x
    transform[1, 3] = pose_msg.position.y
    transform[2, 3] = pose_msg.position.z
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points
    return points @ transform[:3, :3].T + transform[:3, 3]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points

    voxel_indices = np.floor(points / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    return points[np.sort(unique_indices)]


def make_point(xyz: Iterable[float]) -> Point:
    point = Point()
    point.x = float(xyz[0])
    point.y = float(xyz[1])
    point.z = float(xyz[2])
    return point


def robot_color(robot_id: int) -> Tuple[float, float, float]:
    palette = [
        (0.95, 0.35, 0.25),
        (0.20, 0.60, 0.95),
        (0.20, 0.75, 0.35),
        (0.95, 0.75, 0.20),
        (0.80, 0.30, 0.85),
        (0.25, 0.80, 0.80),
    ]
    return palette[robot_id % len(palette)]


class CslamLidarVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('cslam_lidar_visualizer')

        self.declare_parameter('pose_graph_topic', '/cslam/viz/pose_graph')
        self.declare_parameter('keyframe_cloud_topic', '/cslam/viz/keyframe_pointcloud')
        self.declare_parameter('pose_graph_markers_topic', '/cslam/rviz/pose_graph_markers')
        self.declare_parameter('keyframe_clouds_topic', '/cslam/rviz/keyframe_clouds')
        self.declare_parameter('accumulated_map_topic', '/cslam/rviz/accumulated_map')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('accumulated_voxel_size', 0.20)
        self.declare_parameter('node_scale', 0.25)
        self.declare_parameter('edge_width', 0.06)

        self.pose_graph_topic = self.get_parameter('pose_graph_topic').value
        self.keyframe_cloud_topic = self.get_parameter('keyframe_cloud_topic').value
        self.pose_graph_markers_topic = self.get_parameter('pose_graph_markers_topic').value
        self.keyframe_clouds_topic = self.get_parameter('keyframe_clouds_topic').value
        self.accumulated_map_topic = self.get_parameter('accumulated_map_topic').value
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.accumulated_voxel_size = float(self.get_parameter('accumulated_voxel_size').value)
        self.node_scale = float(self.get_parameter('node_scale').value)
        self.edge_width = float(self.get_parameter('edge_width').value)

        self.pose_graphs_by_robot: Dict[int, CachedPoseGraph] = {}
        self.clouds_by_keyframe: Dict[Key, np.ndarray] = {}
        self.current_origin_robot_id: Optional[int] = None
        self.current_frame_id = 'map_0'
        self.last_warned_missing_pose_count = -1

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(PoseGraph, self.pose_graph_topic, self.pose_graph_callback, input_qos)
        self.create_subscription(VizPointCloud, self.keyframe_cloud_topic, self.keyframe_cloud_callback, input_qos)

        self.pose_graph_markers_publisher = self.create_publisher(
            MarkerArray, self.pose_graph_markers_topic, output_qos)
        self.keyframe_clouds_publisher = self.create_publisher(
            PointCloud2, self.keyframe_clouds_topic, output_qos)
        self.accumulated_map_publisher = self.create_publisher(
            PointCloud2, self.accumulated_map_topic, output_qos)

        self.create_timer(self.publish_period_sec, self.publish_visualization)

        self.get_logger().info(
            'CSLAM LiDAR visualizer ready '
            f'pose_graph_topic={self.pose_graph_topic} '
            f'keyframe_cloud_topic={self.keyframe_cloud_topic} '
            f'frame_pattern=map_{{origin_robot_id}}'
        )

    def pose_graph_callback(self, msg: PoseGraph) -> None:
        self.pose_graphs_by_robot[msg.robot_id] = CachedPoseGraph(
            robot_id=msg.robot_id,
            origin_robot_id=msg.origin_robot_id,
            values=list(msg.values),
            edges=list(msg.edges),
        )

    def keyframe_cloud_callback(self, msg: VizPointCloud) -> None:
        key = (msg.robot_id, msg.keyframe_id)
        points = pc2_utils.read_points_numpy_filtered(msg.pointcloud).astype(np.float32)
        if points.ndim == 1:
            points = points.reshape((-1, 3))
        self.clouds_by_keyframe[key] = points[:, :3]

    def compute_active_origin(self) -> Optional[int]:
        if not self.pose_graphs_by_robot:
            return None

        counts = Counter(graph.origin_robot_id for graph in self.pose_graphs_by_robot.values())
        if self.current_origin_robot_id is not None:
            best_count = max(counts.values())
            if counts[self.current_origin_robot_id] == best_count:
                return self.current_origin_robot_id
        return counts.most_common(1)[0][0]

    def build_pose_index(self, origin_robot_id: int) -> Dict[Key, np.ndarray]:
        poses: Dict[Key, np.ndarray] = {}
        for graph in self.pose_graphs_by_robot.values():
            if graph.origin_robot_id != origin_robot_id:
                continue
            for value in graph.values:
                key = (value.key.robot_id, value.key.keyframe_id)
                poses[key] = pose_msg_to_matrix(value.pose)
        return poses

    def build_edge_index(self, origin_robot_id: int) -> List[PoseGraphEdge]:
        edges: List[PoseGraphEdge] = []
        seen = set()
        for graph in self.pose_graphs_by_robot.values():
            if graph.origin_robot_id != origin_robot_id:
                continue
            for edge in graph.edges:
                edge_key = (
                    edge.key_from.robot_id,
                    edge.key_from.keyframe_id,
                    edge.key_to.robot_id,
                    edge.key_to.keyframe_id,
                )
                if edge_key in seen:
                    continue
                seen.add(edge_key)
                edges.append(edge)
        return edges

    def build_marker_array(
        self,
        frame_id: str,
        poses: Dict[Key, np.ndarray],
        edges: List[PoseGraphEdge],
        stamp,
    ) -> MarkerArray:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        nodes = Marker()
        nodes.header.frame_id = frame_id
        nodes.header.stamp = stamp
        nodes.ns = 'nodes'
        nodes.id = 1
        nodes.type = Marker.SPHERE_LIST
        nodes.action = Marker.ADD
        nodes.pose.orientation.w = 1.0
        nodes.scale.x = self.node_scale
        nodes.scale.y = self.node_scale
        nodes.scale.z = self.node_scale

        for key, transform in sorted(poses.items()):
            nodes.points.append(make_point(transform[:3, 3]))
            r, g, b = robot_color(key[0])
            color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            nodes.colors.append(color)
        marker_array.markers.append(nodes)

        odom_edges = Marker()
        odom_edges.header.frame_id = frame_id
        odom_edges.header.stamp = stamp
        odom_edges.ns = 'odometry_edges'
        odom_edges.id = 2
        odom_edges.type = Marker.LINE_LIST
        odom_edges.action = Marker.ADD
        odom_edges.pose.orientation.w = 1.0
        odom_edges.scale.x = self.edge_width
        odom_edges.color.r = 0.75
        odom_edges.color.g = 0.75
        odom_edges.color.b = 0.75
        odom_edges.color.a = 1.0

        inter_robot_edges = Marker()
        inter_robot_edges.header.frame_id = frame_id
        inter_robot_edges.header.stamp = stamp
        inter_robot_edges.ns = 'inter_robot_loop_closures'
        inter_robot_edges.id = 3
        inter_robot_edges.type = Marker.LINE_LIST
        inter_robot_edges.action = Marker.ADD
        inter_robot_edges.pose.orientation.w = 1.0
        inter_robot_edges.scale.x = self.edge_width * 1.25
        inter_robot_edges.color.r = 1.0
        inter_robot_edges.color.g = 0.15
        inter_robot_edges.color.b = 0.15
        inter_robot_edges.color.a = 1.0

        for edge in edges:
            key_from = (edge.key_from.robot_id, edge.key_from.keyframe_id)
            key_to = (edge.key_to.robot_id, edge.key_to.keyframe_id)
            if key_from not in poses or key_to not in poses:
                continue

            p_from = make_point(poses[key_from][:3, 3])
            p_to = make_point(poses[key_to][:3, 3])

            if key_from[0] != key_to[0]:
                inter_robot_edges.points.extend([p_from, p_to])
            elif abs(key_from[1] - key_to[1]) == 1:
                odom_edges.points.extend([p_from, p_to])

        marker_array.markers.extend([odom_edges, inter_robot_edges])
        return marker_array

    def build_transformed_clouds(self, poses: Dict[Key, np.ndarray]) -> Tuple[np.ndarray, int]:
        transformed_clouds: List[np.ndarray] = []
        missing_pose_count = 0

        for key, cloud in self.clouds_by_keyframe.items():
            transform = poses.get(key)
            if transform is None:
                missing_pose_count += 1
                continue
            transformed_clouds.append(transform_points(cloud, transform))

        if missing_pose_count != self.last_warned_missing_pose_count:
            self.last_warned_missing_pose_count = missing_pose_count
            if missing_pose_count > 0:
                self.get_logger().warn(
                    f'{missing_pose_count} cached keyframe clouds still have no optimized pose in the active origin frame.'
                )

        if not transformed_clouds:
            return np.empty((0, 3), dtype=np.float32), missing_pose_count

        return np.concatenate(transformed_clouds, axis=0).astype(np.float32), missing_pose_count

    def publish_visualization(self) -> None:
        origin_robot_id = self.compute_active_origin()
        if origin_robot_id is None:
            return

        self.current_origin_robot_id = origin_robot_id
        self.current_frame_id = f'map_{origin_robot_id}'

        poses = self.build_pose_index(origin_robot_id)
        edges = self.build_edge_index(origin_robot_id)

        stamp = self.get_clock().now().to_msg()
        markers = self.build_marker_array(self.current_frame_id, poses, edges, stamp)
        self.pose_graph_markers_publisher.publish(markers)

        transformed_clouds, _ = self.build_transformed_clouds(poses)

        header = Header()
        header.stamp = stamp
        header.frame_id = self.current_frame_id

        keyframe_clouds_msg = pc2_utils.create_cloud_xyz32(header, transformed_clouds)
        self.keyframe_clouds_publisher.publish(keyframe_clouds_msg)

        accumulated_cloud = voxel_downsample(transformed_clouds, self.accumulated_voxel_size)
        accumulated_map_msg = pc2_utils.create_cloud_xyz32(header, accumulated_cloud)
        self.accumulated_map_publisher.publish(accumulated_map_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CslamLidarVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
