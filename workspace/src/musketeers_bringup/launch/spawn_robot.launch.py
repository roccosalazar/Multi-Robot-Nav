import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
	DeclareLaunchArgument('robot_name', default_value='r0', description='Robot name to spawn (e.g. r0, r1, r2).'),
	DeclareLaunchArgument('x', default_value='0.0', description='Initial x position of the robot.'),
	DeclareLaunchArgument('y', default_value='0.0', description='Initial y position of the robot.'),
	DeclareLaunchArgument('z', default_value='0.35', description='Initial z position of the robot.'),
	DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw of the robot.'),
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
	Spawn one Clearpath robot with a fixed pose.
	"""

	pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
	pkg_slam_evaluation = get_package_share_directory('slam_evaluation')
	robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
	ground_truth_pose_launch = PathJoinSubstitution([pkg_slam_evaluation, 'launch', 'ground_truth_pose.launch.py'])

	robot_spawn = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(robot_spawn_launch),
		launch_arguments={
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'Multi-Robot-Nav/robots', LaunchConfiguration('robot_name')]),
			'world': LaunchConfiguration('world'),
			'rviz': LaunchConfiguration('rviz'),
			'x': LaunchConfiguration('x'),
			'y': LaunchConfiguration('y'),
			'z': LaunchConfiguration('z'),
			'yaw': LaunchConfiguration('yaw'),
			'generate': LaunchConfiguration('generate'),
		}.items()
	)

	ground_truth_pose = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(ground_truth_pose_launch),
		launch_arguments={
			'robot_name': LaunchConfiguration('robot_name'),
			'world': LaunchConfiguration('world'),
			'use_sim_time': LaunchConfiguration('use_sim_time'),
		}.items(),
		condition=IfCondition(LaunchConfiguration('ground_truth_pose')),
	)

	ld = LaunchDescription(ARGUMENTS)
	if _has_nvidia():
		ld.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
		ld.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))
	ld.add_action(robot_spawn)
	ld.add_action(ground_truth_pose)

	return ld
