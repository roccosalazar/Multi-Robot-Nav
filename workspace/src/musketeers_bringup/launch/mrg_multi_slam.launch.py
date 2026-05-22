from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from datetime import datetime, timezone


ARGUMENTS = [
    DeclareLaunchArgument('config', default_value='r012.yaml', description='mrg_slam config file name.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='World frame id used for published SLAM pose.'),
    DeclareLaunchArgument('slam_pose_offset_x', default_value='0.0', description='Output SLAM pose x offset [m].'),
    DeclareLaunchArgument('slam_pose_offset_y', default_value='0.0', description='Output SLAM pose y offset [m].'),
    DeclareLaunchArgument('slam_pose_offset_z', default_value='0.35', description='Output SLAM pose z offset [m].'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame for slam pose tf lookup.'),
    DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame name.'),
    DeclareLaunchArgument('publish_rate', default_value='20.0', description='Publish rate [Hz] for slam pose publisher.'),
    DeclareLaunchArgument('lookup_timeout_sec', default_value='0.1', description='TF lookup timeout [s].'),
    DeclareLaunchArgument('start_evaluation_recorders', default_value='true', choices=['true', 'false'], description='Start aggregate SLAM evaluation CSV recorders.'),
    DeclareLaunchArgument('start_aggregate_recorders', default_value='true', choices=['true', 'false'], description='Internal toggle for the shared multi-robot evaluation recorders.'),
    DeclareLaunchArgument('record_communication', default_value='false', choices=['true', 'false'], description='Record estimated logical inter-robot SLAM communication metrics.'),
    DeclareLaunchArgument('results_root', default_value='', description='Results root. Empty means <Multi-Robot-Nav>/results.'),
    DeclareLaunchArgument('results_run_id', default_value=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'), description='Shared results run id.'),
    DeclareLaunchArgument('evaluation_run_type', default_value='mrg_multi', description='Results subfolder for this launch.'),
    DeclareLaunchArgument('mrg_graph_record_period_sec', default_value='1.0', description='Polling period for MRG graph snapshots.'),
]


def _slam_instance(slam_bringup_launch: PathJoinSubstitution, robot_name: str, x: str, y: str, z: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_bringup_launch),
        launch_arguments={
            'robot_name': robot_name,
            'config': LaunchConfiguration('config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'x': x,
            'y': y,
            'z': z,
            'slam_pose_offset_x': LaunchConfiguration('slam_pose_offset_x'),
            'slam_pose_offset_y': LaunchConfiguration('slam_pose_offset_y'),
            'slam_pose_offset_z': LaunchConfiguration('slam_pose_offset_z'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'lookup_timeout_sec': LaunchConfiguration('lookup_timeout_sec'),
            'start_evaluation_recorders': 'false',
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    pkg_musketeers_bringup = get_package_share_directory('musketeers_bringup')
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')
    slam_bringup_launch = PathJoinSubstitution([pkg_musketeers_bringup, 'launch', 'mrg_single_slam.launch.py'])
    recorders_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'slam_recorders.launch.py'])

    r0_slam = _slam_instance(slam_bringup_launch, 'r0', '-3.0', '0.0', '0.0')
    r1_slam = _slam_instance(slam_bringup_launch, 'r1', '0.0', '0.0', '0.0')
    r2_slam = _slam_instance(slam_bringup_launch, 'r2', '3.0', '0.0', '0.0')
    recorders = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recorders_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_names': 'r0,r1,r2',
            'results_root': LaunchConfiguration('results_root'),
            'run_type': LaunchConfiguration('evaluation_run_type'),
            'run_id': LaunchConfiguration('results_run_id'),
            'graph_mode': 'mrg',
            'record_communication': LaunchConfiguration('record_communication'),
            'mrg_poll_period_sec': LaunchConfiguration('mrg_graph_record_period_sec'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_aggregate_recorders')),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(r0_slam)
    ld.add_action(r1_slam)
    ld.add_action(r2_slam)
    ld.add_action(recorders)
    return ld
