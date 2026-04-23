from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('cslam_lidar_viz'),
        'rviz',
        'cslam_lidar_visualization.rviz',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    visualizer_node = Node(
        package='cslam_lidar_viz',
        executable='cslam_lidar_visualizer',
        name='cslam_lidar_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='cslam_lidar_rviz',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('start_rviz', default_value='true', description='Start RViz2 with the provided config.'),
        visualizer_node,
        rviz_node,
    ])
