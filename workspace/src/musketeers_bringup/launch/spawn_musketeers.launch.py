import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo World'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('generate', default_value='true', choices=['true', 'false'], description='Generate parameters and launch files'),
]


def _has_nvidia() -> bool:
    return (
        os.path.exists('/proc/driver/nvidia/version')
        or os.path.exists('/dev/nvidia0')
    )


def generate_launch_description() -> LaunchDescription:
    """
    Spawn three Clearpath robots with fixed poses.
    """

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    robot1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'clearpath', 'aramis']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '-3.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    robot2_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'clearpath', 'athos']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '0.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    robot3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'clearpath', 'porthos']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '3.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    robot1_tf_relay = Node(
        package='utils',
        executable='tf_prefix_relay',
        name='tf_prefix_relay_aramis',
        parameters=[{'robot_ns': 'aramis'}],
        output='screen',
    )

    robot2_tf_relay = Node(
        package='utils',
        executable='tf_prefix_relay',
        name='tf_prefix_relay_athos',
        parameters=[{'robot_ns': 'athos'}],
        output='screen',
    )

    robot3_tf_relay = Node(
        package='utils',
        executable='tf_prefix_relay',
        name='tf_prefix_relay_porthos',
        parameters=[{'robot_ns': 'porthos'}],
        output='screen',
    )

    robot1_velodyne_relay = Node(
        package='utils',
        executable='velodyne_relay',
        name='velodyne_relay_aramis',
        parameters=[{'robot_ns': 'aramis'}],
        output='screen',
    )

    robot2_velodyne_relay = Node(
        package='utils',
        executable='velodyne_relay',
        name='velodyne_relay_athos',
        parameters=[{'robot_ns': 'athos'}],
        output='screen',
    )

    robot3_velodyne_relay = Node(
        package='utils',
        executable='velodyne_relay',
        name='velodyne_relay_porthos',
        parameters=[{'robot_ns': 'porthos'}],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)
    if _has_nvidia():
        ld.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
        ld.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))

    ld.add_action(robot1_spawn)
    ld.add_action(robot1_tf_relay)
    ld.add_action(robot1_velodyne_relay)

    # Spawn athos after 7 seconds
    ld.add_action(
        TimerAction(
            period=7.0,
            actions=[robot2_spawn, robot2_tf_relay, robot2_velodyne_relay]
        )
    )

    # Spawn porthos after 14 seconds
    ld.add_action(
        TimerAction(
            period=14.0,
            actions=[robot3_spawn, robot3_tf_relay, robot3_velodyne_relay]
        )
    )

    return ld