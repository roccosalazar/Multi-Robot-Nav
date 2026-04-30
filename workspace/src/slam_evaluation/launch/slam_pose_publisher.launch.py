from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='r0', description='Robot namespace.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
    DeclareLaunchArgument('map_frame', default_value='map', description='Map frame for tf lookup.'),
    DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame name.'),
    DeclareLaunchArgument('output_frame_id', default_value='warehouse', description='Published PoseStamped frame_id.'),
    DeclareLaunchArgument('position_offset_x', default_value='0.0', description='Output pose x offset [m].'),
    DeclareLaunchArgument('position_offset_y', default_value='0.0', description='Output pose y offset [m].'),
    DeclareLaunchArgument('position_offset_z', default_value='0.0', description='Output pose z offset [m].'),
    DeclareLaunchArgument('publish_rate', default_value='20.0', description='Publish rate [Hz].'),
    DeclareLaunchArgument('lookup_timeout_sec', default_value='0.1', description='TF lookup timeout [s].'),
]


def generate_launch_description() -> LaunchDescription:
    slam_pose_publisher = Node(
        package='slam_evaluation',
        executable='slam_pose_publisher',
        name='slam_pose_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'position_offset_x': LaunchConfiguration('position_offset_x'),
                'position_offset_y': LaunchConfiguration('position_offset_y'),
                'position_offset_z': LaunchConfiguration('position_offset_z'),
                'output_topic': 'slam/pose',
                'publish_rate': LaunchConfiguration('publish_rate'),
                'lookup_timeout_sec': LaunchConfiguration('lookup_timeout_sec'),
            }
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam_pose_publisher)
    return ld
