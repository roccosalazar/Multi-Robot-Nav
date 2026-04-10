from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('config', default_value='musketeers.yaml', description='mrg_slam config file name.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='World frame id used for published SLAM pose.'),
    DeclareLaunchArgument('slam_pose_offset_x', default_value='0.0', description='Output SLAM pose x offset [m].'),
    DeclareLaunchArgument('slam_pose_offset_y', default_value='0.0', description='Output SLAM pose y offset [m].'),
    DeclareLaunchArgument('slam_pose_offset_z', default_value='0.35', description='Output SLAM pose z offset [m].'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame for slam pose tf lookup.'),
    DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame name.'),
    DeclareLaunchArgument('publish_rate', default_value='20.0', description='Publish rate [Hz] for slam pose publisher.'),
    DeclareLaunchArgument('lookup_timeout_sec', default_value='0.1', description='TF lookup timeout [s].'),
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
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    pkg_musketeers_bringup = get_package_share_directory('musketeers_bringup')
    slam_bringup_launch = PathJoinSubstitution([pkg_musketeers_bringup, 'launch', 'slam_bringup.launch.py'])

    aramis_slam = _slam_instance(slam_bringup_launch, 'aramis', '-3.0', '0.0', '0.0')
    athos_slam = _slam_instance(slam_bringup_launch, 'athos', '0.0', '0.0', '0.0')
    porthos_slam = _slam_instance(slam_bringup_launch, 'porthos', '3.0', '0.0', '0.0')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(aramis_slam)
    ld.add_action(athos_slam)
    ld.add_action(porthos_slam)
    return ld