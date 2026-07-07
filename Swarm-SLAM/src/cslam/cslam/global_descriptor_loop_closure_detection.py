#!/usr/bin/env python
from cslam.algebraic_connectivity_maximization import EdgeInterRobot
import numpy as np

import os
from os.path import join, exists, isfile, realpath, dirname

from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
from cslam.broker import Broker

from cslam_common_interfaces.msg import KeyframeRGB, KeyframePointCloud
from cslam_common_interfaces.msg import (
    GlobalDescriptor, GlobalDescriptors, InterRobotLoopClosure,
    LocalDescriptorsRequest, LocalKeyframeMatch, InterRobotMatch,
    InterRobotMatches)
from diagnostic_msgs.msg import KeyValue
import time
from sortedcontainers import SortedDict

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from cslam.neighbors_manager import NeighborManager
from cslam.simulated_rendezvous import SimulatedRendezvous
from cslam.utils.misc import dict_to_list_chunks

class GlobalDescriptorLoopClosureDetection(object):
    """ Global descriptor matching """

    def __init__(self, params, node):
        """Initialization

        Args:
            params (dict): parameters
            node (ROS 2 node handle): node handle
        """
        self.params = params
        self.node = node
        self.lcm = LoopClosureSparseMatching(params)
        self.received_keyframe_count = 0
        self.computed_descriptor_count = 0
        self.inter_detect_cycles = 0
        self.local_intra_matches_count = 0

        # Place Recognition network setup
        if self.params['frontend.global_descriptor_technique'].lower(
        ) == 'netvlad':
            from cslam.vpr.netvlad import NetVLAD
            self.node.get_logger().info('Using NetVLAD.')
            self.global_descriptor = NetVLAD(self.params, self.node)
            self.keyframe_type = "rgb"
        elif self.params['frontend.global_descriptor_technique'].lower(
        ) == 'scancontext':
            from cslam.lidar_pr.scancontext import ScanContext
            global icp_utils
            import cslam.lidar_pr.icp_utils as icp_utils
            self.node.get_logger().info('Using ScanContext.')
            self.global_descriptor = ScanContext(self.params, self.node)
            self.keyframe_type = "pointcloud"
        else:
            from cslam.vpr.cosplace import CosPlace
            self.node.get_logger().info('Using CosPlace. (default)')
            self.global_descriptor = CosPlace(self.params, self.node)
            self.keyframe_type = "rgb"

        # ROS 2 objects setup
        self.params[
            'frontend.global_descriptors_topic'] = '/cslam/' + self.node.get_parameter(
                'frontend.global_descriptors_topic').value
        self.global_descriptor_publisher = self.node.create_publisher(
            GlobalDescriptors,
            self.params['frontend.global_descriptors_topic'], 100)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalDescriptors,
            self.params['frontend.global_descriptors_topic'],
            self.global_descriptor_callback, 100)

        self.params[
            'frontend.inter_robot_matches_topic'] = '/cslam/' + self.node.get_parameter(
                'frontend.inter_robot_matches_topic').value
        self.inter_robot_matches_publisher = self.node.create_publisher(
            InterRobotMatches,
            self.params['frontend.inter_robot_matches_topic'], 100)
        self.inter_robot_matches_subscriber = self.node.create_subscription(
            InterRobotMatches,
            self.params['frontend.inter_robot_matches_topic'],
            self.inter_robot_matches_callback, 100)

        if self.keyframe_type == "rgb":
            self.receive_keyframe_subscriber = self.node.create_subscription(
                KeyframeRGB, 'cslam/keyframe_data', self.receive_keyframe, 100)
        elif self.keyframe_type == "pointcloud":
            self.receive_keyframe_subscriber = self.node.create_subscription(
                KeyframePointCloud, 'cslam/keyframe_data', self.receive_keyframe,
                100)
        else:
            self.node.get_logger().error("Unknown keyframe type")

        self.local_match_publisher = self.node.create_publisher(
            LocalKeyframeMatch, 'cslam/local_keyframe_match', 100)

        self.pending_inter_robot_edges = {}
        self.confirmed_inter_robot_edge_keys = set()
        self.confirmed_inter_robot_edge_transforms = {}
        self.inter_robot_loop_closure_publisher = self.node.create_publisher(
            InterRobotLoopClosure, '/cslam/inter_robot_loop_closure', 100)
        self.confirmed_inter_robot_loop_closure_subscriber = self.node.create_subscription(
            InterRobotLoopClosure, '/cslam/inter_robot_loop_closure',
            self.receive_confirmed_inter_robot_loop_closure, 100)
        self.receive_inter_robot_loop_closure_subscriber = self.node.create_subscription(
            InterRobotLoopClosure, '/cslam/inter_robot_loop_closure_candidates',
            self.receive_inter_robot_loop_closure, 100)

        self.local_descriptors_request_publishers = {}
        for i in range(self.params['max_nb_robots']):
            self.local_descriptors_request_publishers[
                i] = self.node.create_publisher(
                    LocalDescriptorsRequest,
                    '/r' + str(i) + '/cslam/local_descriptors_request', 100)

        # Listen for changes in node liveliness
        self.neighbor_manager = NeighborManager(
            self.node, self.params)
        self.rendezvous_gate = SimulatedRendezvous(
            self.node,
            self.params.get('evaluation.enable_frontend_simulated_rendezvous', False),
            self.params.get('evaluation.rendezvous_schedule_file', ''),
            self.params['robot_id'],
        )

        self.global_descriptors_buffer = SortedDict()
        self.global_descriptors_timer = self.node.create_timer(
            self.params['frontend.detection_publication_period_sec'],
            self.global_descriptors_timer_callback,
            clock=Clock()
        )  # Note: It is important to use the system clock instead of ROS clock for timers since we are within a TimerAction

        self.inter_robot_matches_buffer = SortedDict()
        self.nb_inter_robot_matches = 0
        self.inter_robot_matches_timer = self.node.create_timer(
            self.params['frontend.detection_publication_period_sec'],
            self.inter_robot_matches_timer_callback,
            clock=Clock()
        )  # Note: It is important to use the system clock instead of ROS clock for timers since we are within a TimerAction

        if self.params["evaluation.enable_logs"]:
            self.log_publisher = self.node.create_publisher(
                KeyValue, 'cslam/log_info', 100)
            self.log_matches_publisher = self.node.create_publisher(
                InterRobotMatches, 'cslam/log_matches', 100)
            self.log_total_successful_matches = 0
            self.log_total_failed_matches = 0
            self.log_total_vertices_transmitted = 0
            self.log_total_matches_selected = 0
            self.log_detection_cumulative_communication = 0
            self.log_total_sparsification_computation_time = 0.0
        
        # Import OpenCV at the end to avoid issue on NVIDIA Jetson Xavier: https://github.com/opencv/opencv/issues/14884#issuecomment-599852128
        global cv2
        import cv2
        global CvBridge
        from cv_bridge import CvBridge

        self.gpu_start_time = time.time() 

        self.node.get_logger().info(
            "[DEBUG_LC_PIPELINE] GLCD ready "
            f"ns={self.node.get_namespace()} "
            f"robot_id={self.params['robot_id']} "
            f"keyframe_type={self.keyframe_type} "
            f"global_descriptor_topic={self.params['frontend.global_descriptors_topic']} "
            f"inter_robot_matches_topic={self.params['frontend.inter_robot_matches_topic']} "
            "inter_robot_loop_candidate_topic=/cslam/inter_robot_loop_closure_candidates "
            "inter_robot_loop_confirmed_topic=/cslam/inter_robot_loop_closure "
            f"inter_loop_consistency={self.params['frontend.enable_inter_loop_consistency_check']} "
            f"inter_loop_min_cluster={self.params['frontend.inter_loop_consistency_min_cluster_size']} "
            f"inter_loop_max_trans_m={self.params['frontend.inter_loop_consistency_max_translation_m']} "
            f"inter_loop_max_rot_deg={self.params['frontend.inter_loop_consistency_max_rotation_deg']} "
            "keyframe_input_topic=cslam/keyframe_data"
        )

    def remote_neighbors_in_range(self):
        """Return remote robot IDs that are currently considered reachable."""
        _, neighbors_in_range_list = self.neighbors_in_range()
        return [
            robot_id for robot_id in neighbors_in_range_list
            if robot_id != self.params['robot_id']
        ]

    def neighbors_in_range(self):
        """Return reachability using the rendezvous schedule when enabled."""
        if self.rendezvous_gate.enabled:
            neighbors_in_range_list = self.rendezvous_gate.robots_alive(
                self.params['max_nb_robots'])
            neighbors_is_in_range = {
                robot_id: robot_id in neighbors_in_range_list
                for robot_id in range(self.params['max_nb_robots'])
            }
            return neighbors_is_in_range, neighbors_in_range_list
        return self.neighbor_manager.check_neighbors_in_range()

    def local_robot_is_in_rendezvous(self):
        """Return whether this robot is allowed to exchange CSLAM data now."""
        return self.rendezvous_gate.is_alive()

    def local_robot_is_broker(self, neighbors_in_range_list):
        """Return whether the local robot is broker for the current rendezvous."""
        if self.rendezvous_gate.enabled:
            return (
                self.params['robot_id'] in neighbors_in_range_list and
                self.params['robot_id'] == min(neighbors_in_range_list)
            )
        return self.neighbor_manager.local_robot_is_broker()

    def add_global_descriptor_to_map(self, embedding, kf_id):
        """ Add global descriptor to matching list

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID
        """

        # Local matching
        self.detect_intra(embedding, kf_id)
        
        # Add for matching
        matches = self.lcm.add_local_global_descriptor(embedding, kf_id)
        

        # Store global descriptor
        msg = GlobalDescriptor()
        msg.keyframe_id = kf_id
        msg.robot_id = self.params['robot_id']
        msg.descriptor = embedding.tolist()
        self.global_descriptors_buffer[kf_id] = msg

        # Store matches
        for match in matches:
            self.inter_robot_matches_buffer[
                self.nb_inter_robot_matches] = match
            self.nb_inter_robot_matches += 1

    def delete_useless_descriptors(self):
        """Deletes global descriptors
           because all other robots have already received them.
        """
        from_kf_id = self.neighbor_manager.useless_descriptors(
            self.global_descriptors_buffer.peekitem(-1)[0])
        if from_kf_id >= self.global_descriptors_buffer.peekitem(0)[0]:
            for k in self.global_descriptors_buffer.keys():
                if k < from_kf_id:
                    del self.global_descriptors_buffer[k]

    def delete_useless_inter_robot_matches(self):
        """Deletes inter_robot_matches
           because all other robots have already received them.
        """
        from_match_id = self.neighbor_manager.useless_matches(
            self.inter_robot_matches_buffer.peekitem(-1)[0])
        if from_match_id >= self.inter_robot_matches_buffer.peekitem(0)[0]:
            for k in self.inter_robot_matches_buffer.keys():
                if k < from_match_id:
                    del self.inter_robot_matches_buffer[k]

    def global_descriptors_timer_callback(self):
        """Publish global descriptors message periodically
        Doesn't publish if the descriptors are already known by neighboring robots
        """
        if not self.local_robot_is_in_rendezvous():
            return
        remote_neighbors = self.remote_neighbors_in_range()
        if len(self.global_descriptors_buffer) > 0 and len(remote_neighbors) > 0:
            from_kf_id = self.neighbor_manager.select_from_which_kf_to_send(
                self.global_descriptors_buffer.peekitem(-1)[0],
                remote_neighbors)

            msgs = dict_to_list_chunks(
                self.global_descriptors_buffer,
                from_kf_id,
                self.params['frontend.detection_publication_max_elems_per_msg']
            )

            for m in msgs:
                global_descriptors = GlobalDescriptors()
                global_descriptors.descriptors = m
                self.global_descriptor_publisher.publish(global_descriptors)
                if self.params["evaluation.enable_logs"]:
                    self.log_detection_cumulative_communication += len(
                        global_descriptors.descriptors) * len(
                            global_descriptors.descriptors[0].descriptor
                        ) * 4  # bytes

            self.delete_useless_descriptors()
            if self.params["evaluation.enable_logs"]:
                self.log_publisher.publish(
                    KeyValue(key="detection_cumulative_communication",
                             value=str(
                                 self.log_detection_cumulative_communication)))

    def edge_to_match(self, edge):
        """Converts an InterRobotEdge to a InterRobotMatch message
           Args: edge (InterRobotEdge)
        """
        msg = InterRobotMatch()
        msg.robot0_id = edge.robot0_id
        msg.robot0_keyframe_id = edge.robot0_keyframe_id
        msg.robot1_id = edge.robot1_id
        msg.robot1_keyframe_id = edge.robot1_keyframe_id
        msg.weight = edge.weight
        return msg

    def inter_robot_matches_timer_callback(self):
        """Publish inter-robot matches message periodically
        Publish only while in rendezvous so sparse candidate matches can reach the broker.
        """
        if not self.local_robot_is_in_rendezvous():
            return
        remote_neighbors = self.remote_neighbors_in_range()
        if len(self.inter_robot_matches_buffer) > 0 and len(remote_neighbors) > 0:
            from_match_idx = self.neighbor_manager.select_from_which_match_to_send(
                self.inter_robot_matches_buffer.peekitem(-1)[0],
                remote_neighbors)

            chuncks = dict_to_list_chunks(
                self.inter_robot_matches_buffer, from_match_idx,
                self.params['frontend.detection_publication_max_elems_per_msg'])

            # The broker already has locally generated direct-pair matches.
            # Non-brokers must still forward theirs so the broker can select them.
            _, neighbors_in_range_list = self.neighbors_in_range()
            if len(neighbors_in_range_list) == 2 and self.local_robot_is_broker(neighbors_in_range_list):
                neighbor_set = set(neighbors_in_range_list)
                chuncks = [[
                    match for match in c
                    if not (match.robot0_id in neighbor_set and match.robot1_id in neighbor_set)
                ] for c in chuncks]
                chuncks = [c for c in chuncks if len(c) > 0]

            # Convert to ROS message
            msgs = []
            for c in chuncks:
                m = []
                for match in c:
                    msg = self.edge_to_match(match)
                    m.append(msg)
                msgs.append(m)

            # Transmit matches
            for m in msgs:
                inter_robot_matches = InterRobotMatches()
                inter_robot_matches.robot_id = self.params['robot_id']
                inter_robot_matches.matches = m
                self.inter_robot_matches_publisher.publish(inter_robot_matches)
                if self.params["evaluation.enable_logs"]:
                    self.log_detection_cumulative_communication += len(
                        inter_robot_matches.matches) * 20  # bytes

            self.delete_useless_inter_robot_matches()
            if self.params["evaluation.enable_logs"]:
                self.log_publisher.publish(
                    KeyValue(key="detection_cumulative_communication",
                             value=str(
                                 self.log_detection_cumulative_communication)))

    def detect_intra(self, embedding, kf_id):
        """ Detect intra-robot loop closures

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID

        Returns:
            list(int): matched keyframes
        """
        if self.params['frontend.enable_intra_robot_loop_closures']:
            kf_match, _ = self.lcm.match_local_loop_closures(embedding, kf_id)
            if kf_match is not None:
                msg = LocalKeyframeMatch()
                msg.keyframe0_id = kf_id
                msg.keyframe1_id = kf_match
                self.local_match_publisher.publish(msg)
                self.local_intra_matches_count += 1
                self.node.get_logger().info(
                    "[DEBUG_LC_PIPELINE] intra-loop candidate "
                    f"from={kf_id} to={kf_match} "
                    f"total_intra_matches={self.local_intra_matches_count}"
                )

    def detect_inter(self):
        """ Detect inter-robot loop closures

        Returns:
            list(int): selected keyframes from other robots to match
        """
        self.inter_detect_cycles += 1
        if not self.local_robot_is_in_rendezvous():
            return
        neighbors_is_in_range, neighbors_in_range_list = self.neighbors_in_range()
        if self.inter_detect_cycles <= 5 or self.inter_detect_cycles % 20 == 0:
            self.node.get_logger().info(
                "[DEBUG_LC_PIPELINE] detect_inter cycle "
                f"count={self.inter_detect_cycles} "
                f"neighbors={neighbors_in_range_list} "
                f"is_broker={self.local_robot_is_broker(neighbors_in_range_list)}"
            )
        #self.node.get_logger().info('Neighbors in range: ' +  str(neighbors_in_range_list))
        remote_neighbors = [
            robot_id for robot_id in neighbors_in_range_list
            if robot_id != self.params['robot_id']
        ]
        # Check if the robot is the broker
        if len(remote_neighbors) > 0 and self.local_robot_is_broker(neighbors_in_range_list):
            if self.params["evaluation.enable_logs"]: start_time = time.time()
            # Find matches that maximize the algebraic connectivity
            selection = self.lcm.select_candidates(
                self.params["frontend.inter_robot_loop_closure_budget"],
                neighbors_is_in_range)
            if len(selection) > 0:
                self.node.get_logger().info(
                    "[DEBUG_LC_PIPELINE] inter-loop candidates selected "
                    f"count={len(selection)}"
                )
            
            # Extract and publish local descriptors
            vertices_info = self.edge_list_to_vertices(selection)
            broker = Broker(selection, neighbors_in_range_list)
            for selected_vertices_set in broker.brokerage(
                    self.params["frontend.use_vertex_cover_selection"]):
                for v in selected_vertices_set:
                    # Call to send publish local descriptors
                    msg = LocalDescriptorsRequest()
                    msg.keyframe_id = v[1]
                    msg.matches_robot_id = vertices_info[v][0]
                    msg.matches_keyframe_id = vertices_info[v][1]
                    self.local_descriptors_request_publishers[v[0]].publish(
                        msg)
                if self.params["evaluation.enable_logs"]:
                    self.log_total_vertices_transmitted += len(
                        selected_vertices_set)
            if self.params["evaluation.enable_logs"]:
                stop_time = time.time()
                self.log_total_sparsification_computation_time += stop_time - start_time
                self.log_total_matches_selected += len(selection)
                self.log_publisher.publish(
                    KeyValue(
                        key="sparsification_cumulative_computation_time",
                        value=str(
                            self.log_total_sparsification_computation_time)))
                self.log_publisher.publish(
                    KeyValue(key="nb_vertices_transmitted",
                             value=str(self.log_total_vertices_transmitted)))
                self.log_publisher.publish(
                    KeyValue(key="nb_matches_selected",
                             value=str(self.log_total_matches_selected)))
                if self.params["evaluation.enable_sparsification_comparison"]:
                    matches = InterRobotMatches()
                    matches.robot_id = self.params["robot_id"]
                    for e in self.lcm.candidate_selector.log_mac_edges:
                        matches.matches.append(self.edge_to_match(e))
                    self.log_matches_publisher.publish(matches)

    def edge_list_to_vertices(self, selection):
        """Extracts the vertices in a list of edges
        Args:
            selection list(EdgeInterRobot): selection of edges
        Returns:
            dict((int, int), list(int), list(int)): Vertices indices with their related vertices
        """
        vertices = {}
        for s in selection:
            key0 = (s.robot0_id, s.robot0_keyframe_id)
            key1 = (s.robot1_id, s.robot1_keyframe_id)
            if key0 in vertices:
                vertices[key0][0].append(s.robot1_id)
                vertices[key0][1].append(s.robot1_keyframe_id)
            else:
                vertices[key0] = [[s.robot1_id], [s.robot1_keyframe_id]]
            if key1 in vertices:
                vertices[key1][0].append(s.robot0_id)
                vertices[key1][1].append(s.robot0_keyframe_id)
            else:
                vertices[key1] = [[s.robot0_id], [s.robot0_keyframe_id]]
        return vertices

    def receive_keyframe(self, msg):
        """Callback to add a keyframe 

        Args:
            msg (cslam_common_interfaces::msg::KeyframeRGB or KeyframePointCloud): Keyframe data
        """
        self.received_keyframe_count += 1
        if self.keyframe_type == "pointcloud":
            points_count = msg.pointcloud.width * msg.pointcloud.height
        else:
            points_count = 0
        if self.received_keyframe_count <= 5 or self.received_keyframe_count % 50 == 0:
            self.node.get_logger().info(
                "[DEBUG_LC_PIPELINE] keyframe received "
                f"count={self.received_keyframe_count} "
                f"id={msg.id} "
                f"points={points_count}"
            )

        # Place recognition descriptor processing
        embedding = []
        if self.keyframe_type == "rgb":
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg.image,
                                            desired_encoding='passthrough')
            embedding = self.global_descriptor.compute_embedding(cv_image)
        elif self.keyframe_type == "pointcloud":
            embedding = self.global_descriptor.compute_embedding(
                icp_utils.ros_pointcloud_to_points(msg.pointcloud))

        self.computed_descriptor_count += 1
        if self.computed_descriptor_count <= 5 or self.computed_descriptor_count % 50 == 0:
            self.node.get_logger().info(
                "[DEBUG_LC_PIPELINE] descriptor computed "
                f"count={self.computed_descriptor_count} "
                f"id={msg.id} "
                f"dim={len(embedding)}"
            )

        self.add_global_descriptor_to_map(embedding, msg.id)

    def global_descriptor_callback(self, msg):
        """Callback for descriptors received from other robots.

        Args:
            msg (cslam_common_interfaces::msg::GlobalDescriptors): descriptors
        """
        if not self.local_robot_is_in_rendezvous():
            return
        if len(msg.descriptors) == 0:
            return
        sender_robot_id = msg.descriptors[0].robot_id
        if sender_robot_id not in self.remote_neighbors_in_range():
            return
        if sender_robot_id != self.params['robot_id']:
            unknown_range = self.neighbor_manager.get_unknown_range(
                msg.descriptors)
            for i in unknown_range:
                match = self.lcm.add_other_robot_global_descriptor(
                    msg.descriptors[i])
                if match is not None:
                    self.inter_robot_matches_buffer[
                        self.nb_inter_robot_matches] = match
                    self.nb_inter_robot_matches += 1

    def inter_robot_matches_callback(self, msg):
        """Callback for inter-robot matches received from other robots.

        Args:
            msg (cslam_common_interfaces::msg::InterRobotMatches): matches
        """
        if not self.local_robot_is_in_rendezvous():
            return
        if msg.robot_id not in self.remote_neighbors_in_range():
            return
        if msg.robot_id != self.params['robot_id']:
            for match in msg.matches:
                edge = EdgeInterRobot(match.robot0_id, match.robot0_keyframe_id, match.robot1_id, match.robot1_keyframe_id, match.weight)
                self.lcm.candidate_selector.add_match(edge)

    def inter_robot_loop_closure_msg_to_edge(self, msg):
        """ Convert a inter-robot loop closure to an edge 
            for algebraic connectivity maximization

        Args:
            msg (cslam_common_interfaces::msg::InterRobotLoopClosure): Inter-robot loop closure

        Returns:
            EdgeInterRobot: inter-robot edge
        """
        return EdgeInterRobot(msg.robot0_id, msg.robot0_keyframe_id,
                              msg.robot1_id, msg.robot1_keyframe_id,
                              self.lcm.candidate_selector.fixed_weight)

    def inter_robot_loop_closure_key(self, msg):
        return (
            msg.robot0_id,
            msg.robot0_keyframe_id,
            msg.robot1_id,
            msg.robot1_keyframe_id,
        )

    def inter_robot_pair_key(self, msg):
        return (
            min(msg.robot0_id, msg.robot1_id),
            max(msg.robot0_id, msg.robot1_id),
        )

    def quaternion_to_matrix(self, q):
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        norm = np.linalg.norm(quat)
        if norm == 0.0 or not np.isfinite(norm):
            return np.eye(3)
        x, y, z, w = quat / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z),
             2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),
             1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),
             2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ])

    def transform_msg_to_matrix(self, transform_msg):
        matrix = np.eye(4)
        matrix[:3, :3] = self.quaternion_to_matrix(transform_msg.rotation)
        matrix[:3, 3] = np.array([
            transform_msg.translation.x,
            transform_msg.translation.y,
            transform_msg.translation.z,
        ])
        return matrix

    def invert_transform_matrix(self, matrix):
        inverse = np.eye(4)
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -(rotation.T @ translation)
        return inverse

    def canonical_inter_robot_measurement(self, msg):
        if getattr(msg, 'has_map_transform', False):
            return self.transform_msg_to_matrix(msg.map_transform)

        # The lidar registration transform is inverted before it is inserted as
        # a GTSAM BetweenFactor. Old publishers may not provide a map-transform
        # hypothesis, so compare candidates in that same measurement convention.
        measurement = self.invert_transform_matrix(
            self.transform_msg_to_matrix(msg.transform))
        if msg.robot0_id <= msg.robot1_id:
            return measurement
        return self.invert_transform_matrix(measurement)

    def transform_residual(self, estimate, reference):
        delta = self.invert_transform_matrix(reference) @ estimate
        translation_error = np.linalg.norm(delta[:3, 3])
        cos_angle = (np.trace(delta[:3, :3]) - 1.0) * 0.5
        rotation_error = np.degrees(
            np.arccos(np.clip(cos_angle, -1.0, 1.0)))
        return float(translation_error), float(rotation_error)

    def inter_robot_transforms_are_consistent(self, transform_a, transform_b):
        translation_error, rotation_error = self.transform_residual(
            transform_a, transform_b)
        return (
            translation_error <=
            self.params['frontend.inter_loop_consistency_max_translation_m'] and
            rotation_error <=
            self.params['frontend.inter_loop_consistency_max_rotation_deg']
        )

    def can_process_inter_robot_loop_candidate(self, msg):
        if not self.local_robot_is_in_rendezvous():
            return False
        _, neighbors_in_range_list = self.neighbors_in_range()
        if (msg.robot0_id not in neighbors_in_range_list or
                msg.robot1_id not in neighbors_in_range_list):
            return False
        return self.local_robot_is_broker(neighbors_in_range_list)

    def can_process_confirmed_inter_robot_loop(self, msg):
        if not self.local_robot_is_in_rendezvous():
            return False
        _, neighbors_in_range_list = self.neighbors_in_range()
        return (
            msg.robot0_id in neighbors_in_range_list and
            msg.robot1_id in neighbors_in_range_list
        )

    def remove_pending_inter_robot_edge(self, msg):
        pair_key = self.inter_robot_pair_key(msg)
        edge_key = self.inter_robot_loop_closure_key(msg)
        if pair_key in self.pending_inter_robot_edges:
            self.pending_inter_robot_edges[pair_key].pop(edge_key, None)

    def remember_confirmed_inter_robot_transform(self, msg):
        pair_key = self.inter_robot_pair_key(msg)
        edge_key = self.inter_robot_loop_closure_key(msg)
        self.confirmed_inter_robot_edge_transforms.setdefault(pair_key, {})[
            edge_key] = self.canonical_inter_robot_measurement(msg)

    def mark_confirmed_inter_robot_loop_closure(self, msg):
        edge_key = self.inter_robot_loop_closure_key(msg)
        if edge_key in self.confirmed_inter_robot_edge_keys:
            return False

        self.confirmed_inter_robot_edge_keys.add(edge_key)
        self.remove_pending_inter_robot_edge(msg)
        self.remember_confirmed_inter_robot_transform(msg)
        self.node.get_logger().info(
            'Confirmed inter-robot loop closure measurement: (' +
            str(msg.robot0_id) + ',' + str(msg.robot0_keyframe_id) +
            ') -> (' + str(msg.robot1_id) + ',' +
            str(msg.robot1_keyframe_id) + ')')
        self.lcm.candidate_selector.candidate_edges_to_fixed(
            [self.inter_robot_loop_closure_msg_to_edge(msg)])

        if self.params["evaluation.enable_logs"]:
            self.log_total_successful_matches += 1
            self.log_publisher.publish(
                KeyValue(key="nb_matches",
                         value=str(self.log_total_successful_matches)))
        return True

    def publish_confirmed_inter_robot_loop_closure(self, msg):
        self.mark_confirmed_inter_robot_loop_closure(msg)
        self.inter_robot_loop_closure_publisher.publish(msg)

    def prune_pending_inter_robot_edges(self, pair_key):
        max_pending = self.params[
            'frontend.inter_loop_consistency_max_pending_edges']
        if max_pending <= 0:
            return
        pending_edges = self.pending_inter_robot_edges.get(pair_key, {})
        while len(pending_edges) > max_pending:
            oldest_key = next(iter(pending_edges))
            pending_edges.pop(oldest_key, None)
            self.node.get_logger().warn(
                "[DEBUG_LC_PIPELINE] dropping old pending inter-loop "
                f"pair={pair_key} edge={oldest_key} "
                f"max_pending={max_pending}")

    def find_consistent_pending_inter_robot_cluster(self, pair_key):
        pending_edges = self.pending_inter_robot_edges.get(pair_key, {})
        min_cluster_size = self.params[
            'frontend.inter_loop_consistency_min_cluster_size']
        if len(pending_edges) < min_cluster_size:
            return []

        records = [
            (edge_key, msg, transform)
            for edge_key, (msg, transform) in pending_edges.items()
        ]
        best_cluster = []
        for seed in records:
            cluster = [seed]
            for candidate in records:
                if candidate[0] == seed[0]:
                    continue
                if all(
                        self.inter_robot_transforms_are_consistent(
                            candidate[2], selected[2])
                        for selected in cluster):
                    cluster.append(candidate)
            if len(cluster) > len(best_cluster):
                best_cluster = cluster

        if len(best_cluster) >= min_cluster_size:
            return best_cluster
        return []

    def is_consistent_with_confirmed_inter_robot_cluster(self, pair_key,
                                                        transform):
        confirmed_transforms = list(
            self.confirmed_inter_robot_edge_transforms.get(pair_key,
                                                          {}).values())
        min_cluster_size = self.params[
            'frontend.inter_loop_consistency_min_cluster_size']
        if len(confirmed_transforms) < min_cluster_size:
            return False
        consistent_count = sum(
            1 for confirmed_transform in confirmed_transforms
            if self.inter_robot_transforms_are_consistent(
                transform, confirmed_transform))
        return consistent_count >= min_cluster_size

    def process_successful_inter_robot_loop_candidate(self, msg):
        if (not self.params['frontend.enable_inter_loop_consistency_check'] or
                self.params[
                    'frontend.inter_loop_consistency_min_cluster_size'] <= 1):
            self.node.get_logger().warn(
                "[DEBUG_LC_PIPELINE] inter-loop consistency check disabled; "
                "publishing candidate directly")
            self.publish_confirmed_inter_robot_loop_closure(msg)
            return

        edge_key = self.inter_robot_loop_closure_key(msg)
        if edge_key in self.confirmed_inter_robot_edge_keys:
            return

        pair_key = self.inter_robot_pair_key(msg)
        if not getattr(msg, 'has_map_transform', False):
            self.node.get_logger().warn(
                "[DEBUG_LC_PIPELINE] inter-loop candidate has no map "
                f"transform hypothesis; falling back to measurement edge={edge_key}")
        transform = self.canonical_inter_robot_measurement(msg)
        if self.is_consistent_with_confirmed_inter_robot_cluster(pair_key,
                                                                transform):
            self.node.get_logger().info(
                "[DEBUG_LC_PIPELINE] inter-loop candidate consistent with "
                f"confirmed cluster pair={pair_key} edge={edge_key}")
            self.publish_confirmed_inter_robot_loop_closure(msg)
            return

        self.pending_inter_robot_edges.setdefault(pair_key, {})[
            edge_key] = (msg, transform)
        self.prune_pending_inter_robot_edges(pair_key)

        cluster = self.find_consistent_pending_inter_robot_cluster(pair_key)
        min_cluster_size = self.params[
            'frontend.inter_loop_consistency_min_cluster_size']
        if len(cluster) >= min_cluster_size:
            self.node.get_logger().info(
                "[DEBUG_LC_PIPELINE] confirmed inter-loop cluster "
                f"pair={pair_key} size={len(cluster)} "
                f"min_size={min_cluster_size} "
                "publishing confirmed measurements")
            for _, cluster_msg, _ in cluster:
                self.publish_confirmed_inter_robot_loop_closure(cluster_msg)
            return

        self.node.get_logger().info(
            "[DEBUG_LC_PIPELINE] pending inter-loop candidate "
            f"pair={pair_key} edge={edge_key} "
            f"pending={len(self.pending_inter_robot_edges[pair_key])} "
            f"min_cluster={min_cluster_size}")

    def process_failed_inter_robot_loop_candidate(self, msg):
        self.remove_pending_inter_robot_edge(msg)
        self.node.get_logger().info(
            'Failed inter-robot loop closure measurement: (' +
            str(msg.robot0_id) + ',' + str(msg.robot0_keyframe_id) +
            ') -> (' + str(msg.robot1_id) + ',' +
            str(msg.robot1_keyframe_id) + ')')
        self.lcm.candidate_selector.remove_candidate_edges(
            [self.inter_robot_loop_closure_msg_to_edge(msg)], failed=True)

        if self.params["evaluation.enable_logs"]:
            self.log_total_failed_matches += 1
            self.log_publisher.publish(
                KeyValue(key="nb_failed_matches",
                         value=str(self.log_total_failed_matches)))

    def receive_confirmed_inter_robot_loop_closure(self, msg):
        """Receive confirmed inter-robot loop closure.

        This keeps candidate-selector state synchronized on every robot while
        the backend consumes the same confirmed topic.
        """
        if not self.can_process_confirmed_inter_robot_loop(msg):
            return
        if msg.success:
            self.mark_confirmed_inter_robot_loop_closure(msg)
        else:
            self.process_failed_inter_robot_loop_candidate(msg)

    def receive_inter_robot_loop_closure(self, msg):
        """Receive computed inter-robot loop closure candidate

        Args:
            msg (cslam_common_interfaces::msg::InterRobotLoopClosure): Inter-robot loop closure
        """
        if not self.can_process_inter_robot_loop_candidate(msg):
            return
        if msg.success:
            self.process_successful_inter_robot_loop_candidate(msg)
        else:
            self.process_failed_inter_robot_loop_candidate(msg)
