from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
	DeclareLaunchArgument('robot_name', default_value='aramis', description='Robot name to spawn (e.g. aramis, athos, porthos).'),
	DeclareLaunchArgument('x', default_value='0.0', description='Initial x position of the robot.'),
	DeclareLaunchArgument('y', default_value='0.0', description='Initial y position of the robot.'),
	DeclareLaunchArgument('z', default_value='0.3', description='Initial z position of the robot.'),
	DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw of the robot.'),
	DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'], description='Start rviz.'),
	DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo World'),
	DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='use_sim_time'),
	DeclareLaunchArgument('generate', default_value='true', choices=['true', 'false'], description='Generate parameters and launch files'),
]


def generate_launch_description() -> LaunchDescription:
	"""
	Spawn one Clearpath robot with a fixed pose.
	"""

	pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
	robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

	robot_spawn = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(robot_spawn_launch),
		launch_arguments={
			'use_sim_time': LaunchConfiguration('use_sim_time'),
			'setup_path': PathJoinSubstitution([EnvironmentVariable('HOME'), 'clearpath', LaunchConfiguration('robot_name')]),
			'world': LaunchConfiguration('world'),
			'rviz': LaunchConfiguration('rviz'),
			'x': LaunchConfiguration('x'),
			'y': LaunchConfiguration('y'),
			'z': LaunchConfiguration('z'),
			'yaw': LaunchConfiguration('yaw'),
			'generate': LaunchConfiguration('generate'),
		}.items()
	)

	robot_tf_relay = Node(
		package='utils',
		executable='tf_prefix_relay',
		name='tf_prefix_relay',
		parameters=[{'robot_ns': LaunchConfiguration('robot_name')}],
		output='screen',
	)

	robot_velodyne_relay = Node(
		package='utils',
		executable='velodyne_relay',
		name='velodyne_relay',
		parameters=[{'robot_ns': LaunchConfiguration('robot_name')}],
		output='screen',
	)

	ld = LaunchDescription(ARGUMENTS)
	ld.add_action(robot_spawn)
	ld.add_action(robot_tf_relay)
	ld.add_action(robot_velodyne_relay)

	return ld
