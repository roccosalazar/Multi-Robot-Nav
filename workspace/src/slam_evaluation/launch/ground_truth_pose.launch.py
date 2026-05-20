import os
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='r0', description='Robot namespace.'),
    DeclareLaunchArgument('world', default_value='auto', description='Gazebo world name or auto.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'], description='Use simulation time.'),
]


def _sanitize_node_name(name: str) -> str:
    sanitized = ''.join(c if (c.isalnum() or c == '_') else '_' for c in name.strip())
    if not sanitized:
        return 'world'
    if sanitized[0].isdigit():
        return f'w_{sanitized}'
    return sanitized


def _resolve_world_names(configured_world: str) -> List[str]:
    world = (configured_world or '').strip()
    if world and world.lower() not in ('auto', 'any', '*'):
        return [world]

    try:
        pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
        worlds_dir = os.path.join(pkg_clearpath_gz, 'worlds')
        worlds = sorted(
            {
                os.path.splitext(entry)[0]
                for entry in os.listdir(worlds_dir)
                if entry.endswith('.sdf')
            }
        )
        if worlds:
            return worlds
    except Exception:
        pass

    return ['warehouse']


def _build_bridges(context, *args, **kwargs):
    configured_world = LaunchConfiguration('world').perform(context)
    world_names = _resolve_world_names(configured_world)
    bridges = []

    for world_name in world_names:
        safe_world = _sanitize_node_name(world_name)
        bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'ground_truth_pose_bridge_{safe_world}',
                namespace=LaunchConfiguration('robot_name'),
                output='screen',
                arguments=[
                    f'/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                ],
                remappings=[
                    (
                        f'/world/{world_name}/dynamic_pose/info',
                        'ground_truth/pose_raw',
                    ),
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        )

    return bridges


def generate_launch_description() -> LaunchDescription:
    bridges = OpaqueFunction(function=_build_bridges)

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
    ld.add_action(bridges)
    ld.add_action(extractor)
    return ld
