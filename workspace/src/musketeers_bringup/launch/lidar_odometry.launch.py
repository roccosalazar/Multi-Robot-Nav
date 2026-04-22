from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='aramis', description='Robot namespace.'),
    DeclareLaunchArgument('config', default_value='scanmatching.yaml', description='mrg_slam config file name.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
]


def generate_launch_description() -> LaunchDescription:
    pkg_mrg_slam = get_package_share_directory('mrg_slam')
    mrg_slam_launch = PathJoinSubstitution([pkg_mrg_slam, 'launch', 'mrg_slam_v2.launch.py'])

    scanmatching = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mrg_slam_launch),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'model_namespace': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_map2odom_publisher': 'false',
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(scanmatching)
    return ld