import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo World'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('generate', default_value='false', choices=['true', 'false'], description='Generate parameters and launch files'),
]


def _has_nvidia() -> bool:
    return (
        os.path.exists('/proc/driver/nvidia/version')
        or os.path.exists('/dev/nvidia0')
    )


def generate_launch_description() -> LaunchDescription:
    """
    Spawn two Clearpath robots (aramis and athos) with fixed poses.
    """

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    aramis_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', 'aramis']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '-3.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    athos_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', 'athos']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '0.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    if _has_nvidia():
        ld.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
        ld.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))

    ld.add_action(aramis_spawn)

    ld.add_action(
        TimerAction(
            period=7.0,
            actions=[athos_spawn]
        )
    )

    return ld