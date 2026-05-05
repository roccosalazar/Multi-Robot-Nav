from ament_index_python.packages import get_package_share_directory

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
        default_value='cslam_lidar.yaml',
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
        default_value='/cslam_rviz/pose_graph_markers',
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
        default_value='/cslam_rviz/map_points',
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
]


def _get_launch_value(context: LaunchContext, name: str) -> str:
    """
    Resolve a launch argument to a string.
    Args:
        context: Current launch context.
        name: Launch argument name.
    Return:
        Resolved launch argument value.
    """
    return context.perform_substitution(LaunchConfiguration(name))


def _get_launch_bool(context: LaunchContext, name: str) -> bool:
    """
    Resolve a launch argument to a boolean.
    Args:
        context: Current launch context.
        name: Launch argument name.
    Return:
        Boolean value parsed from the launch argument.
    """
    value = _get_launch_value(context, name).strip().lower()

    if value in ('true', '1', 'yes', 'on'):
        return True

    if value in ('false', '0', 'no', 'off'):
        return False

    raise ValueError(f"Invalid boolean value for launch argument '{name}': {value}")


def _get_launch_int(context: LaunchContext, name: str) -> int:
    """
    Resolve a launch argument to an integer.
    Args:
        context: Current launch context.
        name: Launch argument name.
    Return:
        Integer value parsed from the launch argument.
    """
    return int(_get_launch_value(context, name))


def _get_launch_float(context: LaunchContext, name: str) -> float:
    """
    Resolve a launch argument to a float.
    Args:
        context: Current launch context.
        name: Launch argument name.
    Return:
        Float value parsed from the launch argument.
    """
    return float(_get_launch_value(context, name))


def _create_lidar_odometry_action() -> IncludeLaunchDescription:
    """
    Create the external mrg_slam launch include used for scan-matching odometry.
    The returned action starts mrg_slam_v2.launch.py with the robot namespace and config.
    """
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
    """
    Create the core CSLAM nodes that were previously inside swarm_slam_bringup.launch.py.
    The returned list contains loop closure detection, map manager, pose graph manager and TF bridge.
    """
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
        executable='lidar_handler_node.py',
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
                'odom_topic': 'scan_matching_odometry/odom',
                'max_anchor_time_diff_sec': 2.0,
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
    """
    Create a delayed action for the core CSLAM nodes.
    The delay preserves the startup ordering previously used by single_cslam.launch.py.
    """
    delay_sec = _get_launch_float(context, 'swarm_slam_delay_sec')
    cslam_nodes = _create_cslam_nodes(context)

    return [
        TimerAction(
            period=delay_sec,
            actions=cslam_nodes,
        )
    ]


def _delayed_pose_graph_viewer_action(context: LaunchContext, pkg_slam_evaluation: str) -> list[TimerAction]:
    """
    Create a delayed action for the pose graph viewer launch.
    The returned action starts the external cslam_pose_graph_viewer.launch.py when enabled.
    """
    cslam_pose_graph_viewer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'cslam_pose_graph_viewer.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': _get_launch_value(context, 'use_sim_time'),
            'input_topic': _get_launch_value(context, 'pose_graph_viewer_input_topic'),
            'output_topic': _get_launch_value(context, 'pose_graph_viewer_output_topic'),
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
    """
    Create a delayed action for the keyframe cloud viewer node.
    The returned action starts the viewer node when cloud visualization is enabled.
    """
    cslam_keyframe_cloud_viewer_node = Node(
        package='slam_evaluation',
        executable='cslam_keyframe_cloud_viewer',
        name='cslam_keyframe_cloud_viewer',
        output='screen',
        parameters=[
            {
                'use_sim_time': _get_launch_bool(context, 'use_sim_time'),
                'pose_graph_topic': _get_launch_value(context, 'keyframe_pose_graph_topic'),
                'keyframe_cloud_topic': _get_launch_value(context, 'keyframe_cloud_topic'),
                'keyframe_odom_topic': _get_launch_value(context, 'keyframe_odom_topic'),
                'output_topic': _get_launch_value(context, 'keyframe_cloud_output_topic'),
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
    """
    Generate a single flattened launch description for one CSLAM robot.
    It starts external mrg_slam odometry, direct CSLAM nodes and visualization tools.
    """
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')

    launch_description = LaunchDescription(ARGUMENTS)
    launch_description.add_action(_create_lidar_odometry_action())
    launch_description.add_action(OpaqueFunction(function=_delayed_cslam_nodes_action))
    launch_description.add_action(
        OpaqueFunction(function=_delayed_pose_graph_viewer_action, args=[pkg_slam_evaluation])
    )
    launch_description.add_action(OpaqueFunction(function=_delayed_keyframe_cloud_viewer_action))

    return launch_description
