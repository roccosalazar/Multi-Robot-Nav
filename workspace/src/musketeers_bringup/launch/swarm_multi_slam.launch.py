from typing import Union

from ament_index_python.packages import get_package_share_directory

from datetime import datetime, timezone
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ROBOT_CONFIGS = [
    ('r0', '0'),
    ('r1', '1'),
    ('r2', '2'),
]

MAX_NB_ROBOTS = str(len(ROBOT_CONFIGS))


ARGUMENTS = [
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
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation clock for all nodes.',
    ),
    DeclareLaunchArgument(
        'launch_prefix_cslam',
        default_value='',
        description='Optional debug prefix for core CSLAM nodes.',
    ),
    DeclareLaunchArgument(
        'enable_simulated_rendezvous',
        default_value='true',
        choices=['true', 'false'],
        description='Enable simulated rendezvous evaluation mode.',
    ),
    DeclareLaunchArgument(
        'enable_frontend_simulated_rendezvous',
        default_value='true',
        choices=['true', 'false'],
        description='Also gate frontend descriptor and loop-closure communication by rendezvous.',
    ),
    DeclareLaunchArgument(
        'rendezvous_schedule_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('musketeers_bringup'),
            'config',
            'rendezvous_test.config',
        ]),
        description='Path to the rendezvous schedule file.',
    ),
    DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level for CSLAM nodes.',
    ),
    DeclareLaunchArgument(
        'start_graph_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Start one namespaced pose graph viewer per robot instance.',
    ),
    DeclareLaunchArgument(
        'start_cloud_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Start one namespaced keyframe cloud viewer per robot instance.',
    ),
    DeclareLaunchArgument(
        'start_merged_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Start one MRG-style cached merged viewer per robot instance.',
    ),
    DeclareLaunchArgument(
        'swarm_slam_delay_sec',
        default_value='2.0',
        description='Delay before starting the core CSLAM nodes inside each robot instance.',
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
        'robot_centric_viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Aggregate CSLAM visualization topics into one MRG-style robot-centric view.',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_node_scale',
        default_value='0.30',
        description='Sphere diameter for pose-graph nodes in meters.',
    ),
    DeclareLaunchArgument(
        'pose_graph_viewer_edge_width',
        default_value='0.05',
        description='Line width for pose-graph edges in meters.',
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
        'merged_viewer_delay_sec',
        default_value='6.0',
        description='Delay before starting the cached merged CSLAM viewers.',
    ),
    DeclareLaunchArgument(
        'start_evaluation_recorders',
        default_value='true',
        choices=['true', 'false'],
        description='Start aggregate SLAM evaluation CSV recorders.',
    ),
    DeclareLaunchArgument(
        'record_communication',
        default_value='true',
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
        default_value='swarm_multi',
        description='Results subfolder for this launch.',
    ),
]


def _create_swarm_single_slam_instance(
    swarm_single_slam_launch: PathJoinSubstitution,
    robot_name: str,
    robot_id: str,
    start_graph_viewer: Union[LaunchConfiguration, str],
    start_cloud_viewer: Union[LaunchConfiguration, str],
) -> GroupAction:
    """
    Create one robot instance by including swarm_single_slam.launch.py.
    Args:
        swarm_single_slam_launch: Path to the single-robot flattened launch file.
        robot_name: Robot namespace.
        robot_id: Numeric robot identifier as a string.
        start_graph_viewer: Launch value used to enable or disable the graph viewer.
        start_cloud_viewer: Launch value used to enable or disable the cloud viewer.
    Return:
        IncludeLaunchDescription action for one robot instance.
    """
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(swarm_single_slam_launch),
                launch_arguments={
                    'robot_name': robot_name,
                    'robot_id': robot_id,
                    'max_nb_robots': MAX_NB_ROBOTS,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'config_path': LaunchConfiguration('config_path'),
                    'config_file': LaunchConfiguration('config_file'),
                    'lidar_odometry_config': LaunchConfiguration('lidar_odometry_config'),
                    'launch_prefix_cslam': LaunchConfiguration('launch_prefix_cslam'),
                    'enable_simulated_rendezvous': LaunchConfiguration('enable_simulated_rendezvous'),
                    'enable_frontend_simulated_rendezvous': LaunchConfiguration('enable_frontend_simulated_rendezvous'),
                    'rendezvous_schedule_file': LaunchConfiguration('rendezvous_schedule_file'),
                    'log_level': LaunchConfiguration('log_level'),
                    'start_graph_viewer': start_graph_viewer,
                    'start_cloud_viewer': start_cloud_viewer,
                    'start_merged_viewer': LaunchConfiguration('start_merged_viewer'),
                    'swarm_slam_delay_sec': LaunchConfiguration('swarm_slam_delay_sec'),
                    'pose_graph_viewer_delay_sec': LaunchConfiguration('pose_graph_viewer_delay_sec'),
                    'keyframe_cloud_viewer_delay_sec': LaunchConfiguration('keyframe_cloud_viewer_delay_sec'),
                    'merged_viewer_delay_sec': LaunchConfiguration('merged_viewer_delay_sec'),
                    'pose_graph_viewer_input_topic': LaunchConfiguration('pose_graph_viewer_input_topic'),
                    'pose_graph_viewer_output_topic': LaunchConfiguration('pose_graph_viewer_output_topic'),
                    'robot_centric_viewer': LaunchConfiguration('robot_centric_viewer'),
                    'pose_graph_viewer_node_scale': LaunchConfiguration('pose_graph_viewer_node_scale'),
                    'pose_graph_viewer_edge_width': LaunchConfiguration('pose_graph_viewer_edge_width'),
                    'keyframe_pose_graph_topic': LaunchConfiguration('keyframe_pose_graph_topic'),
                    'keyframe_cloud_topic': LaunchConfiguration('keyframe_cloud_topic'),
                    'keyframe_odom_topic': f'/{robot_name}/cslam/keyframe_odom',
                    'keyframe_cloud_output_topic': LaunchConfiguration('keyframe_cloud_output_topic'),
                    'keyframe_point_scale': LaunchConfiguration('keyframe_point_scale'),
                    'max_points_per_keyframe': LaunchConfiguration('max_points_per_keyframe'),
                    'keyframe_stride': LaunchConfiguration('keyframe_stride'),
                    'keyframe_cloud_voxel_size': LaunchConfiguration('keyframe_cloud_voxel_size'),
                    'keyframe_cloud_publish_period_sec': LaunchConfiguration('keyframe_cloud_publish_period_sec'),
                    'initialize_anchor_from_first_pose': 'true',
                    'start_evaluation_recorders': 'false',
                }.items(),
            )
        ],
    )


def generate_launch_description() -> LaunchDescription:
    """
    Generate a multi-robot launch description using swarm_single_slam.launch.py.
    It starts one flattened single-robot SLAM pipeline for each configured robot.
    """
    pkg_musketeers_bringup = get_package_share_directory('musketeers_bringup')
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')
    swarm_single_slam_launch = PathJoinSubstitution([
        pkg_musketeers_bringup,
        'launch',
        'swarm_single_slam.launch.py',
    ])
    recorders_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'slam_recorders.launch.py'])

    launch_description = LaunchDescription(ARGUMENTS)

    for robot_name, robot_id in ROBOT_CONFIGS:
        launch_description.add_action(
            _create_swarm_single_slam_instance(
                swarm_single_slam_launch=swarm_single_slam_launch,
                robot_name=robot_name,
                robot_id=robot_id,
                start_graph_viewer=LaunchConfiguration('start_graph_viewer'),
                start_cloud_viewer=LaunchConfiguration('start_cloud_viewer'),
            )
        )

    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recorders_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_names': ','.join(robot_name for robot_name, _ in ROBOT_CONFIGS),
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
    )

    return launch_description
