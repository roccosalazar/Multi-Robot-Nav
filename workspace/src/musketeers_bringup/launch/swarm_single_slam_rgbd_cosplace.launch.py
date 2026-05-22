from ament_index_python.packages import get_package_share_directory

from datetime import datetime, timezone
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_name',
        default_value='r0',
        description='Robot namespace used for odometry and CSLAM nodes.',
    ),
    DeclareLaunchArgument('robot_id', default_value='0', description='Robot numeric identifier.'),
    DeclareLaunchArgument('max_nb_robots', default_value='1', description='Total number of robots.'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation clock for all nodes in this pipeline.',
    ),
    DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory('musketeers_bringup'),
            'config',
        ]),
        description='Directory containing the CSLAM YAML config file.',
    ),
    DeclareLaunchArgument(
        'config_file',
        default_value='cslam_rgbd_cosplace.yaml',
        description='CSLAM YAML config file name.',
    ),
    DeclareLaunchArgument(
        'lidar_odometry_config',
        default_value='scanmatching.yaml',
        description='mrg_slam config file name used by mrg_slam_v2.launch.py.',
    ),
    DeclareLaunchArgument(
        'launch_prefix_cslam',
        default_value='',
        description='Optional debug prefix for core CSLAM nodes.',
    ),
    DeclareLaunchArgument(
        'enable_simulated_rendezvous',
        default_value='false',
        choices=['true', 'false'],
        description='Enable simulated rendezvous evaluation mode.',
    ),
    DeclareLaunchArgument(
        'rendezvous_schedule_file',
        default_value='',
        description='Path to the rendezvous schedule file.',
    ),
    DeclareLaunchArgument('log_level', default_value='info', description='ROS log level for CSLAM nodes.'),
    DeclareLaunchArgument(
        'start_graph_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Start the pose graph viewer.',
    ),
    DeclareLaunchArgument(
        'start_cloud_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Start the keyframe cloud viewer.',
    ),
    DeclareLaunchArgument(
        'swarm_slam_delay_sec',
        default_value='2.0',
        description='Delay before starting the core CSLAM nodes.',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_delay_sec',
        default_value='4.0',
        description='Delay before starting the pose graph viewer.',
    ),
    DeclareLaunchArgument(
        'keyframe_cloud_viewer_delay_sec',
        default_value='5.0',
        description='Delay before starting the keyframe cloud viewer.',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_input_topic',
        default_value='/cslam/viz/pose_graph',
        description='Input PoseGraph topic for cslam_pose_graph_viewer.launch.py.',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_output_topic',
        default_value='cslam_rviz/pose_graph_markers',
        description='Output MarkerArray topic for the pose graph viewer.',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_node_scale',
        default_value='0.30',
        description='Sphere diameter for pose-graph nodes [m].',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_edge_width',
        default_value='0.05',
        description='Line width for pose-graph edges [m].',
    ),
    DeclareLaunchArgument(
        'keyframe_pose_graph_topic',
        default_value='/cslam/viz/pose_graph',
        description='Pose graph topic consumed by cslam_keyframe_cloud_viewer.',
    ),
    DeclareLaunchArgument(
        'keyframe_cloud_topic',
        default_value='/cslam/viz/keyframe_pointcloud',
        description='Keyframe point cloud topic consumed by cslam_keyframe_cloud_viewer.',
    ),
    DeclareLaunchArgument(
        'keyframe_odom_topic',
        default_value=['/', LaunchConfiguration('robot_name'), '/cslam/keyframe_odom'],
        description='Keyframe odometry topic consumed by cslam_keyframe_cloud_viewer.',
    ),
    DeclareLaunchArgument(
        'keyframe_cloud_output_topic',
        default_value='cslam_rviz/map_points',
        description='Output PointCloud2 topic for the CSLAM global map viewer.',
    ),
    DeclareLaunchArgument(
        'keyframe_point_scale',
        default_value='0.08',
        description='Legacy parameter kept for backward compatibility with the old marker-based viewer.',
    ),
    DeclareLaunchArgument(
        'max_points_per_keyframe',
        default_value='0',
        description='Maximum stored points per keyframe before global-map fusion. Use 0 to disable downsampling.',
    ),
    DeclareLaunchArgument(
        'keyframe_stride',
        default_value='1',
        description='Use only one keyframe cloud every N keyframes. Use 1 to use all keyframes.',
    ),
    DeclareLaunchArgument(
        'keyframe_cloud_voxel_size',
        default_value='0.10',
        description='Voxel size in meters used to downsample the fused global CSLAM map.',
    ),
    DeclareLaunchArgument(
        'keyframe_cloud_publish_period_sec',
        default_value='0.5',
        description='Publish period in seconds for the fused global CSLAM map.',
    ),
    DeclareLaunchArgument(
        'start_evaluation_recorders',
        default_value='true',
        choices=['true', 'false'],
        description='Start SLAM evaluation CSV recorders.',
    ),
    DeclareLaunchArgument(
        'record_communication',
        default_value='false',
        choices=['true', 'false'],
        description='Record estimated logical inter-robot SLAM communication metrics.',
    ),
    DeclareLaunchArgument(
        'results_root',
        default_value='',
        description='Results root. Empty means <Multi-Robot-Nav>/results.',
    ),
    DeclareLaunchArgument(
        'results_run_id',
        default_value=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'),
        description='Shared results run id.',
    ),
    DeclareLaunchArgument(
        'evaluation_run_type',
        default_value='swarm_single',
        description='Results subfolder for this launch.',
    ),
]


def _get_launch_value(context: LaunchContext, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _get_launch_bool(context: LaunchContext, name: str) -> bool:
    value = _get_launch_value(context, name).strip().lower()

    if value in ('true', '1', 'yes', 'on'):
        return True

    if value in ('false', '0', 'no', 'off'):
        return False

    raise ValueError(f"Invalid boolean value for launch argument '{name}': {value}")


def _get_launch_int(context: LaunchContext, name: str) -> int:
    return int(_get_launch_value(context, name))


def _get_launch_float(context: LaunchContext, name: str) -> float:
    return float(_get_launch_value(context, name))


def _create_lidar_odometry_action() -> IncludeLaunchDescription:
    pkg_mrg_slam = get_package_share_directory('mrg_slam')
    mrg_slam_launch = PathJoinSubstitution([pkg_mrg_slam, 'launch', 'mrg_slam_v2.launch.py'])

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mrg_slam_launch),
        launch_arguments={
            'config': LaunchConfiguration('lidar_odometry_config'),
            'model_namespace': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_map2odom_publisher': 'false',
        }.items(),
    )


def _create_cslam_nodes(context: LaunchContext) -> list[Node]:
    config = context.perform_substitution(
        PathJoinSubstitution([
            LaunchConfiguration('config_path'),
            LaunchConfiguration('config_file'),
        ])
    )

    namespace = _get_launch_value(context, 'robot_name')

    common_params = {
        'robot_id': _get_launch_int(context, 'robot_id'),
        'max_nb_robots': _get_launch_int(context, 'max_nb_robots'),
        'use_sim_time': _get_launch_bool(context, 'use_sim_time'),
    }

    log_args = ['--ros-args', '--log-level', _get_launch_value(context, 'log_level')]

    loop_detection_node = Node(
        package='cslam',
        executable='loop_closure_detection_node.py',
        name='cslam_loop_closure_detection',
        parameters=[config, common_params],
        namespace=namespace,
        arguments=log_args,
    )

    map_manager_node = Node(
        package='cslam',
        executable='map_manager',
        name='cslam_map_manager',
        parameters=[config, common_params],
        prefix=_get_launch_value(context, 'launch_prefix_cslam'),
        namespace=namespace,
        arguments=log_args,
    )

    pose_graph_manager_node = Node(
        package='cslam',
        executable='pose_graph_manager',
        name='cslam_pose_graph_manager',
        parameters=[
            config,
            common_params,
            {
                'evaluation.enable_simulated_rendezvous': _get_launch_bool(
                    context,
                    'enable_simulated_rendezvous',
                ),
                'evaluation.rendezvous_schedule_file': _get_launch_value(
                    context,
                    'rendezvous_schedule_file',
                ),
            },
        ],
        prefix=_get_launch_value(context, 'launch_prefix_cslam'),
        namespace=namespace,
        arguments=log_args,
    )

    cslam_tf_bridge_node = Node(
        package='slam_evaluation',
        executable='cslam_odom_tf_bridge',
        name='cslam_odom_tf_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'use_sim_time': _get_launch_bool(context, 'use_sim_time'),
                'cslam_pose_topic': 'cslam/current_pose_estimate',
                'reference_frames_topic': 'cslam/reference_frames',
                'odom_topic': 'scan_matching_odometry/odom',
                'robot_id': _get_launch_int(context, 'robot_id'),
                'local_map_frame_id': f"robot{_get_launch_int(context, 'robot_id')}_map",
                'use_local_map_frame': True,
                'max_anchor_time_diff_sec': 2.0,
                'initialize_anchor_from_first_pose': False,
            }
        ],
        arguments=log_args,
    )

    return [
        loop_detection_node,
        map_manager_node,
        pose_graph_manager_node,
        cslam_tf_bridge_node,
    ]


def _delayed_cslam_nodes_action(context: LaunchContext) -> list[TimerAction]:
    delay_sec = _get_launch_float(context, 'swarm_slam_delay_sec')
    cslam_nodes = _create_cslam_nodes(context)

    return [
        TimerAction(
            period=delay_sec,
            actions=cslam_nodes,
        )
    ]


def _delayed_pose_graph_viewer_action(context: LaunchContext, pkg_slam_evaluation: str) -> list[TimerAction]:
    cslam_pose_graph_viewer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'cslam_pose_graph_viewer.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': _get_launch_value(context, 'use_sim_time'),
            'namespace': _get_launch_value(context, 'robot_name'),
            'input_topic': _get_launch_value(context, 'pose_graph_viewer_input_topic'),
            'output_topic': _get_launch_value(context, 'pose_graph_viewer_output_topic'),
            'robot_id': _get_launch_value(context, 'robot_id'),
            'node_scale': _get_launch_value(context, 'pose_graph_viewer_node_scale'),
            'edge_width': _get_launch_value(context, 'pose_graph_viewer_edge_width'),
        }.items(),
    )

    delay_sec = _get_launch_float(context, 'pose_graph_viewer_delay_sec')

    return [
        TimerAction(
            period=delay_sec,
            actions=[cslam_pose_graph_viewer_launch],
            condition=IfCondition(LaunchConfiguration('start_graph_viewer')),
        )
    ]


def _delayed_keyframe_cloud_viewer_action(context: LaunchContext) -> list[TimerAction]:
    cslam_keyframe_cloud_viewer_node = Node(
        package='slam_evaluation',
        executable='cslam_keyframe_cloud_viewer',
        name='cslam_keyframe_cloud_viewer',
        namespace=_get_launch_value(context, 'robot_name'),
        output='screen',
        parameters=[
            {
                'use_sim_time': _get_launch_bool(context, 'use_sim_time'),
                'pose_graph_topic': _get_launch_value(context, 'keyframe_pose_graph_topic'),
                'keyframe_cloud_topic': _get_launch_value(context, 'keyframe_cloud_topic'),
                'keyframe_odom_topic': _get_launch_value(context, 'keyframe_odom_topic'),
                'output_topic': _get_launch_value(context, 'keyframe_cloud_output_topic'),
                'robot_id': _get_launch_int(context, 'robot_id'),
                'point_scale': _get_launch_float(context, 'keyframe_point_scale'),
                'max_points_per_keyframe': _get_launch_int(context, 'max_points_per_keyframe'),
                'keyframe_stride': _get_launch_int(context, 'keyframe_stride'),
                'voxel_size': _get_launch_float(context, 'keyframe_cloud_voxel_size'),
                'publish_period_sec': _get_launch_float(context, 'keyframe_cloud_publish_period_sec'),
            }
        ],
    )

    delay_sec = _get_launch_float(context, 'keyframe_cloud_viewer_delay_sec')

    return [
        TimerAction(
            period=delay_sec,
            actions=[cslam_keyframe_cloud_viewer_node],
            condition=IfCondition(LaunchConfiguration('start_cloud_viewer')),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')
    recorders_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'slam_recorders.launch.py'])
    recorders = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recorders_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_names': LaunchConfiguration('robot_name'),
            'results_root': LaunchConfiguration('results_root'),
            'run_type': LaunchConfiguration('evaluation_run_type'),
            'run_id': LaunchConfiguration('results_run_id'),
            'graph_mode': 'swarm',
            'record_communication': LaunchConfiguration('record_communication'),
            'swarm_pose_graph_topics': LaunchConfiguration('pose_graph_viewer_input_topic'),
            'swarm_record_merged_graphs': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_evaluation_recorders')),
    )

    launch_description = LaunchDescription(ARGUMENTS)
    launch_description.add_action(_create_lidar_odometry_action())
    launch_description.add_action(OpaqueFunction(function=_delayed_cslam_nodes_action))
    launch_description.add_action(
        OpaqueFunction(function=_delayed_pose_graph_viewer_action, args=[pkg_slam_evaluation])
    )
    launch_description.add_action(OpaqueFunction(function=_delayed_keyframe_cloud_viewer_action))
    launch_description.add_action(recorders)

    return launch_description
