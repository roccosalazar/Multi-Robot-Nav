from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from datetime import datetime, timezone


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='r0', description='Robot namespace.'),
    DeclareLaunchArgument('config', default_value='r0.yaml', description='mrg_slam config file name.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='World frame id used for published SLAM pose.'),
    DeclareLaunchArgument('x', default_value='0.0', description='Initial SLAM x position.'),
    DeclareLaunchArgument('y', default_value='0.0', description='Initial SLAM y position.'),
    DeclareLaunchArgument('z', default_value='0.0', description='Initial SLAM z position.'),
    DeclareLaunchArgument('slam_pose_offset_x', default_value='0.0', description='Output SLAM pose x offset [m].'),
    DeclareLaunchArgument('slam_pose_offset_y', default_value='0.0', description='Output SLAM pose y offset [m].'),
    DeclareLaunchArgument('slam_pose_offset_z', default_value='0.35', description='Output SLAM pose z offset [m].'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame for slam pose tf lookup.'),
    DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame name.'),
    DeclareLaunchArgument('publish_rate', default_value='20.0', description='Publish rate [Hz] for slam pose publisher.'),
    DeclareLaunchArgument('lookup_timeout_sec', default_value='0.1', description='TF lookup timeout [s].'),
    DeclareLaunchArgument('start_evaluation_recorders', default_value='true', choices=['true', 'false'], description='Start SLAM evaluation CSV recorders.'),
    DeclareLaunchArgument('record_communication', default_value='true', choices=['true', 'false'], description='Record estimated logical inter-robot SLAM communication metrics.'),
    DeclareLaunchArgument('results_root', default_value='', description='Results root. Empty means <Multi-Robot-Nav>/results.'),
    DeclareLaunchArgument('results_run_id', default_value=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'), description='Shared results run id.'),
    DeclareLaunchArgument('evaluation_run_type', default_value='mrg', description='Results subfolder for this launch.'),
    DeclareLaunchArgument('mrg_graph_record_period_sec', default_value='1.0', description='Polling period for MRG graph snapshots.'),
]


def generate_launch_description() -> LaunchDescription:
    pkg_mrg_slam = get_package_share_directory('mrg_slam')
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')
    mrg_slam_launch = PathJoinSubstitution([pkg_mrg_slam, 'launch', 'mrg_slam.launch.py'])
    recorders_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'slam_recorders.launch.py'])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mrg_slam_launch),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'model_namespace': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }.items(),
    )

    slam_pose_publisher = Node(
        package='slam_evaluation',
        executable='slam_pose_publisher',
        name='slam_pose_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'output_frame_id': LaunchConfiguration('world'),
                'position_offset_x': LaunchConfiguration('slam_pose_offset_x'),
                'position_offset_y': LaunchConfiguration('slam_pose_offset_y'),
                'position_offset_z': LaunchConfiguration('slam_pose_offset_z'),
                'output_topic': 'slam/pose',
                'publish_rate': LaunchConfiguration('publish_rate'),
                'lookup_timeout_sec': LaunchConfiguration('lookup_timeout_sec'),
            }
        ],
    )

    recorders = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recorders_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_names': LaunchConfiguration('robot_name'),
            'results_root': LaunchConfiguration('results_root'),
            'run_type': LaunchConfiguration('evaluation_run_type'),
            'run_id': LaunchConfiguration('results_run_id'),
            'graph_mode': 'mrg',
            'record_communication': LaunchConfiguration('record_communication'),
            'mrg_poll_period_sec': LaunchConfiguration('mrg_graph_record_period_sec'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_evaluation_recorders')),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam)
    ld.add_action(slam_pose_publisher)
    ld.add_action(recorders)
    return ld
