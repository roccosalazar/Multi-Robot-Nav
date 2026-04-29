import os
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


RobotConfig = Tuple[str, str, str, str, str]


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo World.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
    DeclareLaunchArgument('generate', default_value='false', choices=['true', 'false'], description='Generate parameters and launch files.'),
    DeclareLaunchArgument(
        'trio',
        default_value='musketeers',
        choices=['musketeers', 'swarm_slam'],
        description='Robot trio to launch. Use musketeers for aramis/athos/porthos, swarm_slam for r0/r1/r2.',
    ),
    DeclareLaunchArgument(
        'ground_truth_pose',
        default_value='true',
        choices=['true', 'false'],
        description='Forward and filter Gazebo dynamic pose to PoseStamped in /<robot_name>/ground_truth/pose.',
    ),
]


def _has_nvidia() -> bool:
    """
    Check whether the system appears to have an NVIDIA GPU available.
    Return True when NVIDIA-related files/devices are detected.
    """
    return os.path.exists('/proc/driver/nvidia/version') or os.path.exists('/dev/nvidia0')


def _get_robot_configs(trio: str) -> List[RobotConfig]:
    """
    Return robot names and spawn poses for the selected trio.
    Arguments:
        trio: Name of the robot trio selected from the launch argument.
    Return:
        A list of robot configurations as tuples: name, x, y, z, yaw.
    """
    trio_configs: Dict[str, List[RobotConfig]] = {
        'musketeers': [
            ('aramis', '-3.0', '0.0', '0.35', '0.0'),
            ('athos', '0.0', '0.0', '0.35', '0.0'),
            ('porthos', '3.0', '0.0', '0.35', '0.0'),
        ],
        'swarm_slam': [
            ('r0', '-3.0', '0.0', '0.35', '0.0'),
            ('r1', '0.0', '0.0', '0.35', '0.0'),
            ('r2', '3.0', '0.0', '0.35', '0.0'),
        ],
    }

    if trio not in trio_configs:
        raise ValueError(f"Unsupported trio '{trio}'.")

    return trio_configs[trio]


def _create_robot_actions(
    robot_name: str,
    x: str,
    y: str,
    z: str,
    yaw: str,
    robot_spawn_launch: PathJoinSubstitution,
    ground_truth_pose_launch: PathJoinSubstitution,
) -> List[Action]:
    """
    Create launch actions for spawning one robot and optionally starting its ground-truth pose node.
    Arguments:
        robot_name: Name and namespace/setup folder of the robot.
        x: Initial x position.
        y: Initial y position.
        z: Initial z position.
        yaw: Initial yaw angle.
        robot_spawn_launch: Path to the Clearpath robot spawn launch file.
        ground_truth_pose_launch: Path to the ground-truth pose launch file.
    Return:
        A list containing the robot spawn action and the ground-truth pose action.
    """
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', robot_name]),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'generate': LaunchConfiguration('generate'),
        }.items(),
    )

    robot_ground_truth_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ground_truth_pose_launch),
        launch_arguments={
            'robot_name': robot_name,
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('ground_truth_pose')),
    )

    return [robot_spawn, robot_ground_truth_pose]


def _launch_setup(context: LaunchContext) -> List[Action]:
    """
    Build robot launch actions after resolving launch configurations at runtime.
    Arguments:
        context: Launch context used to evaluate LaunchConfiguration values.
    Return:
        A list of actions that spawn the selected trio with fixed delays.
    """
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_slam_evaluation = get_package_share_directory('slam_evaluation')

    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
    ground_truth_pose_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'ground_truth_pose.launch.py'])

    trio = LaunchConfiguration('trio').perform(context)
    robot_configs = _get_robot_configs(trio)

    actions: List[Action] = []

    for index, robot_config in enumerate(robot_configs):
        robot_name, x, y, z, yaw = robot_config
        robot_actions = _create_robot_actions(robot_name, x, y, z, yaw, robot_spawn_launch, ground_truth_pose_launch)

        if index == 0:
            actions.extend(robot_actions)
        else:
            actions.append(
                TimerAction(
                    period=7.0 * index,
                    actions=robot_actions,
                )
            )

    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Spawn three Clearpath robots with selectable naming scheme and fixed spawn delays.
    """
    launch_description = LaunchDescription(ARGUMENTS)

    if _has_nvidia():
        launch_description.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
        launch_description.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))

    launch_description.add_action(OpaqueFunction(function=_launch_setup))

    return launch_description