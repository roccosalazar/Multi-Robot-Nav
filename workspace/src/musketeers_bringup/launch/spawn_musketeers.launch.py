import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo World'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('generate', default_value='false', choices=['true', 'false'], description='Generate parameters and launch files'),
    DeclareLaunchArgument(
        'ground_truth_pose',
        default_value='true',
        choices=['true', 'false'],
        description='Forward and filter Gazebo dynamic pose to PoseStamped in /<robot_name>/ground_truth/pose.',
    ),
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
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')

    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
    ground_truth_pose_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'ground_truth_pose.launch.py'])

    robot1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', 'aramis']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '-3.0',
            'y': '0.0',
            'z': '0.35',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    robot1_ground_truth_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ground_truth_pose_launch),
        launch_arguments={
            'robot_name': 'aramis',
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('ground_truth_pose')),
    )

    robot2_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', 'athos']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '0.0',
            'y': '0.0',
            'z': '0.35',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    robot2_ground_truth_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ground_truth_pose_launch),
        launch_arguments={
            'robot_name': 'athos',
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('ground_truth_pose')),
    )

    robot3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', 'porthos']),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': '3.0',
            'y': '0.0',
            'z': '0.35',
            'yaw': '0.0',
            'generate': LaunchConfiguration('generate'),
        }.items()
    )

    robot3_ground_truth_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ground_truth_pose_launch),
        launch_arguments={
            'robot_name': 'porthos',
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('ground_truth_pose')),
    )

    ld = LaunchDescription(ARGUMENTS)
    if _has_nvidia():
        ld.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
        ld.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))

    ld.add_action(robot1_spawn)
    ld.add_action(robot1_ground_truth_pose)

    # Spawn athos after 7 seconds
    ld.add_action(
        TimerAction(
            period=7.0,
            actions=[robot2_spawn, robot2_ground_truth_pose]
        )
    )

    # Spawn porthos after 14 seconds
    ld.add_action(
        TimerAction(
            period=14.0,
            actions=[robot3_spawn, robot3_ground_truth_pose]
        )
    )

    return ld