from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument(
        "robot_names",
        default_value="r0,r1,r2",
        description="Comma-separated robot namespaces.",
    ),
    DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution([
            get_package_share_directory("musketeers_bringup"),
            "config",
        ]),
        description="Directory containing the FAST_LIO YAML config file.",
    ),
    DeclareLaunchArgument(
        "config_file",
        default_value="fastlio_vlp16_sim.yaml",
        description="FAST_LIO YAML config file name.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation clock.",
    ),
    DeclareLaunchArgument(
        "lidar_topic_suffix",
        default_value="/sensors/lidar3d_0/points",
        description="Robot-relative LiDAR PointCloud2 topic suffix.",
    ),
    DeclareLaunchArgument(
        "imu_topic_suffix",
        default_value="/sensors/imu_0/data",
        description="Robot-relative IMU topic suffix.",
    ),
    DeclareLaunchArgument("scan_line", default_value="16", description="LiDAR vertical channels."),
    DeclareLaunchArgument("scan_rate", default_value="50", description="LiDAR scan rate [Hz]."),
    DeclareLaunchArgument(
        "broadcast_tf",
        default_value="true",
        choices=["true", "false"],
        description="Broadcast robot odom->base_link TF for each FAST_LIO instance.",
    ),
    DeclareLaunchArgument(
        "use_body_to_base_tf",
        default_value="true",
        choices=["true", "false"],
        description="Apply static robot TF from FAST_LIO body frame to base_link.",
    ),
    DeclareLaunchArgument(
        "lookup_timeout_sec",
        default_value="0.05",
        description="TF lookup timeout for body->base correction.",
    ),
    DeclareLaunchArgument("log_level", default_value="info", description="ROS log level."),
]


def _get_launch_value(context: LaunchContext, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _create_fast_lio_instances(context: LaunchContext) -> list[IncludeLaunchDescription]:
    pkg_musketeers_bringup = get_package_share_directory("musketeers_bringup")
    single_fast_lio_launch = PathJoinSubstitution([
        pkg_musketeers_bringup,
        "launch",
        "single_fast_lio.launch.py",
    ])

    robot_names = [
        robot_name.strip()
        for robot_name in _get_launch_value(context, "robot_names").split(",")
        if robot_name.strip()
    ]
    lidar_topic_suffix = _get_launch_value(context, "lidar_topic_suffix")
    imu_topic_suffix = _get_launch_value(context, "imu_topic_suffix")

    actions: list[IncludeLaunchDescription] = []
    for robot_name in robot_names:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_fast_lio_launch),
                launch_arguments={
                    "robot_name": robot_name,
                    "config_path": LaunchConfiguration("config_path"),
                    "config_file": LaunchConfiguration("config_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "lidar_topic": f"/{robot_name}{lidar_topic_suffix}",
                    "imu_topic": f"/{robot_name}{imu_topic_suffix}",
                    "scan_line": LaunchConfiguration("scan_line"),
                    "scan_rate": LaunchConfiguration("scan_rate"),
                    "raw_odom_topic": f"/{robot_name}/fastlio/raw_odom",
                    "odom_topic": f"/{robot_name}/fastlio/odom",
                    "odom_frame_id": f"{robot_name}/odom",
                    "base_frame_id": f"{robot_name}/base_link",
                    "fastlio_body_frame_id": f"{robot_name}/imu_0_link",
                    "fastlio_internal_tf_topic": f"/{robot_name}/fastlio/internal_tf",
                    "fastlio_internal_tf_static_topic": f"/{robot_name}/fastlio/internal_tf_static",
                    "broadcast_tf": LaunchConfiguration("broadcast_tf"),
                    "use_body_to_base_tf": LaunchConfiguration("use_body_to_base_tf"),
                    "lookup_timeout_sec": LaunchConfiguration("lookup_timeout_sec"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription(ARGUMENTS)
    launch_description.add_action(OpaqueFunction(function=_create_fast_lio_instances))
    return launch_description
