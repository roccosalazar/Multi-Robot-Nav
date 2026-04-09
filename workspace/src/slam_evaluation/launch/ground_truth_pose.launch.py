from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='aramis', description='Robot namespace.'),
    DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo world name.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
]


def generate_launch_description() -> LaunchDescription:
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_pose_bridge',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        arguments=[
            [
                '/world/',
                LaunchConfiguration('world'),
                '/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
        ],
        remappings=[
            (
                ['/world/', LaunchConfiguration('world'), '/dynamic_pose/info'],
                'ground_truth/pose_raw',
            ),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    extractor = Node(
        package='slam_evaluation',
        executable='ground_truth_pose_extractor',
        name='ground_truth_pose_extractor',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': 'ground_truth/pose_raw',
                'output_topic': 'ground_truth/pose',
                'target_entity_name': [LaunchConfiguration('robot_name'), '/robot'],
                'output_frame_id': LaunchConfiguration('world'),
            }
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(bridge)
    ld.add_action(extractor)
    return ld
