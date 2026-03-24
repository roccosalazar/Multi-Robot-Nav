from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo World'),
]

def generate_launch_description() -> LaunchDescription:
    """
    Launch the Gazebo world.
    """

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    gz_sim_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    return ld