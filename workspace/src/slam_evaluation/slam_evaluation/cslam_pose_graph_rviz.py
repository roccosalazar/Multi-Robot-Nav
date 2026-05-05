#!/usr/bin/env python3

from typing import Dict, List, Tuple

import rclpy
from cslam_common_interfaces.msg import PoseGraph
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class CSLAMPoseGraphRViz(Node):
    """Convert CSLAM PoseGraph messages into RViz MarkerArray markers."""

    def __init__(self) -> None:
        super().__init__('cslam_pose_graph_rviz')

        self.declare_parameter('input_topic', '/cslam/viz/pose_graph')
        self.declare_parameter('output_topic', '/cslam_rviz/pose_graph_markers')
        self.declare_parameter('node_scale', 0.30)
        self.declare_parameter('edge_width', 0.05)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.node_scale = float(self.get_parameter('node_scale').value)
        self.edge_width = float(self.get_parameter('edge_width').value)

        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(MarkerArray, self.output_topic, pub_qos)
        self.pose_graph_sub = self.create_subscription(
            PoseGraph,
            self.input_topic,
            self._pose_graph_callback,
            sub_qos,
        )

        self._last_frame_id = None

        self.get_logger().info(
            "Publishing RViz markers from '%s' to '%s' using frame 'robot{origin_robot_id}_map'."
            % (self.input_topic, self.output_topic)
        )

    def _pose_graph_callback(self, msg: PoseGraph) -> None:
        frame_id = self._frame_id_from_msg(msg)
        if frame_id != self._last_frame_id:
            self.get_logger().info(
                "Using frame '%s' derived from origin_robot_id=%d."
                % (frame_id, msg.origin_robot_id)
            )
            self._last_frame_id = frame_id

        timestamp = self.get_clock().now().to_msg()
        keyframe_positions: Dict[Tuple[int, int], Point] = {}
        node_points_by_robot: Dict[int, List[Point]] = {}

        for value in msg.values:
            point = Point(
                x=float(value.pose.position.x),
                y=float(value.pose.position.y),
                z=float(value.pose.position.z),
            )
            key = (int(value.key.robot_id), int(value.key.keyframe_id))
            keyframe_positions[key] = point
            node_points_by_robot.setdefault(key[0], []).append(point)

        intra_robot_odom_segments: List[Point] = []
        intra_robot_loop_segments: List[Point] = []
        inter_robot_segments: List[Point] = []

        for edge in msg.edges:
            key_from = (int(edge.key_from.robot_id), int(edge.key_from.keyframe_id))
            key_to = (int(edge.key_to.robot_id), int(edge.key_to.keyframe_id))

            point_from = keyframe_positions.get(key_from)
            point_to = keyframe_positions.get(key_to)
            if point_from is None or point_to is None:
                continue

            if edge.key_from.robot_id == edge.key_to.robot_id:
                if abs(key_from[1] - key_to[1]) == 1:
                    intra_robot_odom_segments.extend([point_from, point_to])
                else:
                    intra_robot_loop_segments.extend([point_from, point_to])
            else:
                inter_robot_segments.extend([point_from, point_to])

        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker())

        marker_id = 0
        for robot_id in sorted(node_points_by_robot):
            marker_array.markers.append(
                self._make_nodes_marker(
                    marker_id=marker_id,
                    frame_id=frame_id,
                    timestamp=timestamp,
                    robot_id=robot_id,
                    points=node_points_by_robot[robot_id],
                )
            )
            marker_id += 1

        marker_array.markers.append(
            self._make_edges_marker(
                marker_id=marker_id,
                frame_id=frame_id,
                timestamp=timestamp,
                namespace='intra_robot_odom_edges',
                points=intra_robot_odom_segments,
                color=self._rgba(0.15, 0.65, 1.0, 0.95),
            )
        )
        marker_id += 1

        marker_array.markers.append(
            self._make_edges_marker(
                marker_id=marker_id,
                frame_id=frame_id,
                timestamp=timestamp,
                namespace='intra_robot_loop_edges',
                points=intra_robot_loop_segments,
                color=self._rgba(0.62, 0.22, 0.87, 0.98),
            )
        )
        marker_id += 1

        marker_array.markers.append(
            self._make_edges_marker(
                marker_id=marker_id,
                frame_id=frame_id,
                timestamp=timestamp,
                namespace='inter_robot_edges',
                points=inter_robot_segments,
                color=self._rgba(1.0, 0.35, 0.10, 0.95),
            )
        )

        self.marker_pub.publish(marker_array)

    def _frame_id_from_msg(self, msg: PoseGraph) -> str:
        return f'robot{int(msg.origin_robot_id)}_map'

    def _delete_all_marker(self) -> Marker:
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def _make_nodes_marker(
        self,
        marker_id: int,
        frame_id: str,
        timestamp,
        robot_id: int,
        points: List[Point],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = 'nodes'
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.node_scale
        marker.scale.y = self.node_scale
        marker.scale.z = self.node_scale
        marker.color = self._robot_color(robot_id)
        marker.points = points
        return marker

    def _make_edges_marker(
        self,
        marker_id: int,
        frame_id: str,
        timestamp,
        namespace: str,
        points: List[Point],
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.edge_width
        marker.color = color
        marker.points = points
        return marker

    def _robot_color(self, robot_id: int) -> ColorRGBA:
        palette = [
            (0.10, 0.70, 1.00),
            (0.10, 0.85, 0.35),
            (1.00, 0.75, 0.15),
            (0.95, 0.30, 0.30),
            (0.70, 0.45, 1.00),
            (0.20, 0.90, 0.90),
        ]
        red, green, blue = palette[robot_id % len(palette)]
        return self._rgba(red, green, blue, 0.95)

    @staticmethod
    def _rgba(red: float, green: float, blue: float, alpha: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = red
        color.g = green
        color.b = blue
        color.a = alpha
        return color


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CSLAMPoseGraphRViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
