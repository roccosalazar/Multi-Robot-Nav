from datetime import datetime, timezone

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time.',
    ),
    DeclareLaunchArgument(
        'robot_names',
        default_value='r0',
        description='Comma-separated robot names to record.',
    ),
    DeclareLaunchArgument(
        'results_root',
        default_value='',
        description='Results root. Empty means <Multi-Robot-Nav>/results.',
    ),
    DeclareLaunchArgument(
        'run_type',
        default_value='run',
        description=(
            'Results subfolder name such as mrg, swarm_single, '
            'or multi_data_run.'
        ),
    ),
    DeclareLaunchArgument(
        'run_id',
        default_value=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'),
        description=(
            'Per-run results id shared by recorder nodes.'
        ),
    ),
    DeclareLaunchArgument(
        'start_ground_truth_recorder',
        default_value='true',
        choices=['true', 'false'],
        description='Start the ground-truth trajectory CSV recorder.',
    ),
    DeclareLaunchArgument(
        'start_pose_graph_recorder',
        default_value='true',
        choices=['true', 'false'],
        description='Start the pose-graph snapshot recorder.',
    ),
    DeclareLaunchArgument(
        'graph_mode',
        default_value='auto',
        choices=['auto', 'swarm', 'mrg'],
        description='Graph source mode.',
    ),
    DeclareLaunchArgument(
        'swarm_pose_graph_topics',
        default_value='/cslam/viz/pose_graph',
        description='Comma-separated Swarm PoseGraph topics to record.',
    ),
    DeclareLaunchArgument(
        'swarm_record_merged_graphs',
        default_value='true',
        choices=['true', 'false'],
        description='Also record /<robot>/cslam/viz/merged_pose_graph topics.',
    ),
    DeclareLaunchArgument(
        'mrg_poll_period_sec',
        default_value='1.0',
        description='Polling period for MRG graph services.',
    ),
]


def generate_launch_description() -> LaunchDescription:
    ground_truth_recorder = Node(
        package='slam_evaluation',
        executable='ground_truth_trajectory_recorder',
        name='ground_truth_trajectory_recorder',
        output='log',
        condition=IfCondition(
            LaunchConfiguration('start_ground_truth_recorder')
        ),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_names': LaunchConfiguration('robot_names'),
                'results_root': LaunchConfiguration('results_root'),
                'run_type': LaunchConfiguration('run_type'),
                'run_id': ParameterValue(
                    LaunchConfiguration('run_id'),
                    value_type=str,
                ),
                'pose_topic_template': '/{robot_name}/ground_truth/pose',
            }
        ],
    )

    pose_graph_recorder = Node(
        package='slam_evaluation',
        executable='pose_graph_recorder',
        name='pose_graph_recorder',
        output='log',
        condition=IfCondition(
            LaunchConfiguration('start_pose_graph_recorder')
        ),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mode': LaunchConfiguration('graph_mode'),
                'robot_names': LaunchConfiguration('robot_names'),
                'results_root': LaunchConfiguration('results_root'),
                'run_type': LaunchConfiguration('run_type'),
                'run_id': ParameterValue(
                    LaunchConfiguration('run_id'),
                    value_type=str,
                ),
                'swarm_pose_graph_topics': LaunchConfiguration(
                    'swarm_pose_graph_topics'
                ),
                'swarm_record_merged_graphs': LaunchConfiguration(
                    'swarm_record_merged_graphs'
                ),
                'mrg_poll_period_sec': LaunchConfiguration(
                    'mrg_poll_period_sec'
                ),
            }
        ],
    )

    launch_description = LaunchDescription(ARGUMENTS)
    launch_description.add_action(ground_truth_recorder)
    launch_description.add_action(pose_graph_recorder)
    return launch_description
