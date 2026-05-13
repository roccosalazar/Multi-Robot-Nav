from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time.',
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Optional namespace for the viewer node and relative output topic.',
    ),
    DeclareLaunchArgument(
        'node_name',
        default_value='cslam_pose_graph_rviz',
        description='Name of the RViz pose graph viewer node.',
    ),
    DeclareLaunchArgument(
        'input_topic',
        default_value='/cslam/viz/pose_graph',
        description='Input PoseGraph topic.',
    ),
    DeclareLaunchArgument(
        'output_topic',
        default_value='cslam_rviz/pose_graph_markers',
        description='Output MarkerArray topic.',
    ),
    DeclareLaunchArgument(
        'robot_id',
        default_value='-1',
        description='Only publish pose graphs from this origin robot id. Use -1 for all robots.',
    ),
    DeclareLaunchArgument(
        'node_scale',
        default_value='0.30',
        description='Sphere diameter for pose-graph nodes [m].',
    ),
    DeclareLaunchArgument(
        'edge_width',
        default_value='0.05',
        description='Line width for pose-graph edges [m].',
    ),
]


def generate_launch_description() -> LaunchDescription:
    viewer_node = Node(
        package='slam_evaluation',
        executable='cslam_pose_graph_rviz',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'robot_id': LaunchConfiguration('robot_id'),
                'node_scale': LaunchConfiguration('node_scale'),
                'edge_width': LaunchConfiguration('edge_width'),
            }
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(viewer_node)
    return ld
