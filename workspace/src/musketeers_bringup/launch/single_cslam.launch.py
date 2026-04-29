from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
        description='LiDAR odometry config file name passed to lidar_odometry.launch.py.',
    ),
    DeclareLaunchArgument(
        'launch_prefix_cslam',
        default_value='',
        description='Optional debug prefix for core CSLAM nodes.',
    ),
    DeclareLaunchArgument(
        'enable_simulated_rendezvous',
        default_value='false',
        description='Enable simulated rendezvous evaluation mode.',
    ),
    DeclareLaunchArgument(
        'rendezvous_schedule_file',
        default_value='',
        description='Path to the rendezvous schedule file.',
    ),
    DeclareLaunchArgument('log_level', default_value='info', description='ROS log level for CSLAM nodes.'),
    DeclareLaunchArgument(
        'swarm_slam_delay_sec',
        default_value='2.0',
        description='Delay before starting the core Swarm-SLAM bringup.',
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
        default_value='/cslam_rviz/keyframe_cloud_markers',
        description='Output MarkerArray topic for the keyframe cloud viewer.',
    ),
    DeclareLaunchArgument(
        'keyframe_point_scale',
        default_value='0.08',
        description='Marker point size for the keyframe cloud viewer [m].',
    ),
    DeclareLaunchArgument(
        'max_points_per_keyframe',
        default_value='0',
        description='Maximum rendered points per keyframe. Use 0 to disable downsampling.',
    ),
]


def generate_launch_description() -> LaunchDescription:
    pkg_musketeers_bringup = get_package_share_directory('musketeers_bringup')
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')

    lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_musketeers_bringup, 'launch', 'lidar_odometry.launch.py'])
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'config': LaunchConfiguration('lidar_odometry_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    swarm_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_musketeers_bringup, 'launch', 'swarm_slam_bringup.launch.py'])
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'robot_id': LaunchConfiguration('robot_id'),
            'max_nb_robots': LaunchConfiguration('max_nb_robots'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
            'launch_prefix_cslam': LaunchConfiguration('launch_prefix_cslam'),
            'enable_simulated_rendezvous': LaunchConfiguration('enable_simulated_rendezvous'),
            'rendezvous_schedule_file': LaunchConfiguration('rendezvous_schedule_file'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    cslam_pose_graph_viewer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'cslam_pose_graph_viewer.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('pose_graph_viewer_input_topic'),
            'output_topic': LaunchConfiguration('pose_graph_viewer_output_topic'),
            'node_scale': LaunchConfiguration('pose_graph_viewer_node_scale'),
            'edge_width': LaunchConfiguration('pose_graph_viewer_edge_width'),
        }.items(),
    )

    cslam_keyframe_cloud_viewer_node = Node(
        package='slam_evaluation',
        executable='cslam_keyframe_cloud_viewer',
        name='cslam_keyframe_cloud_viewer',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'pose_graph_topic': LaunchConfiguration('keyframe_pose_graph_topic'),
                'keyframe_cloud_topic': LaunchConfiguration('keyframe_cloud_topic'),
                'keyframe_odom_topic': LaunchConfiguration('keyframe_odom_topic'),
                'output_topic': LaunchConfiguration('keyframe_cloud_output_topic'),
                'point_scale': LaunchConfiguration('keyframe_point_scale'),
                'max_points_per_keyframe': LaunchConfiguration('max_points_per_keyframe'),
            }
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(lidar_odometry_launch)
    ld.add_action(
        TimerAction(
            period=LaunchConfiguration('swarm_slam_delay_sec'),
            actions=[swarm_slam_launch],
        )
    )
    ld.add_action(
        TimerAction(
            period=LaunchConfiguration('pose_graph_viewer_delay_sec'),
            actions=[cslam_pose_graph_viewer_launch],
        )
    )
    ld.add_action(
        TimerAction(
            period=LaunchConfiguration('keyframe_cloud_viewer_delay_sec'),
            actions=[cslam_keyframe_cloud_viewer_node],
        )
    )
    return ld
