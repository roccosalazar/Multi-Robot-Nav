from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'robot_name',
        default_value='r0',
        description='Robot namespace used for all CSLAM nodes.',
    ),
    DeclareLaunchArgument('robot_id', default_value='0', description='Robot numeric identifier.'),
    DeclareLaunchArgument('max_nb_robots', default_value='1', description='Total number of robots.'),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation clock for CSLAM nodes.',
    ),
    DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory('musketeers_bringup'),
            'config',
        ]),
        description='Directory containing the CSLAM YAML config file.',
    ),
    DeclareLaunchArgument(
        'config_file',
        default_value='cslam_lidar.yaml',
        description='CSLAM YAML config file name.',
    ),
    DeclareLaunchArgument(
        'launch_prefix_cslam',
        default_value='',
        description='Optional debug prefix (e.g. "xterm -e gdb -ex run --args").',
    ),
    DeclareLaunchArgument(
        'enable_simulated_rendezvous',
        default_value='false',
        description='Enable simulated rendezvous evaluation mode.',
    ),
    DeclareLaunchArgument(
        'rendezvous_schedule_file',
        default_value='',
        description='Path to the rendezvous schedule file.',
    ),
    DeclareLaunchArgument('log_level', default_value='info', description='ROS log level for CSLAM nodes.'),
]


def generate_launch_description() -> LaunchDescription:

    config = PathJoinSubstitution([
        LaunchConfiguration('config_path'),
        LaunchConfiguration('config_file'),
    ])
    namespace = LaunchConfiguration('robot_name')
    common_params = {
        'robot_id': LaunchConfiguration('robot_id'),
        'max_nb_robots': LaunchConfiguration('max_nb_robots'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    log_args = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    loop_detection_node = Node(
        package='cslam',
        executable='loop_closure_detection_node.py',
        name='cslam_loop_closure_detection',
        parameters=[config, common_params],
        namespace=namespace,
        arguments=log_args,
    )

    map_manager_node = Node(
        package='cslam',
        executable='lidar_handler_node.py',
        name='cslam_map_manager',
        parameters=[config, common_params],
        prefix=LaunchConfiguration('launch_prefix_cslam'),
        namespace=namespace,
        arguments=log_args,
    )

    pose_graph_manager_node = Node(
        package='cslam',
        executable='pose_graph_manager',
        name='cslam_pose_graph_manager',
        parameters=[
            config,
            common_params,
            {
                'evaluation.enable_simulated_rendezvous': LaunchConfiguration('enable_simulated_rendezvous'),
                'evaluation.rendezvous_schedule_file': LaunchConfiguration('rendezvous_schedule_file'),
            },
        ],
        prefix=LaunchConfiguration('launch_prefix_cslam'),
        namespace=namespace,
        arguments=log_args,
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(loop_detection_node)
    ld.add_action(map_manager_node)
    ld.add_action(pose_graph_manager_node)
    return ld
