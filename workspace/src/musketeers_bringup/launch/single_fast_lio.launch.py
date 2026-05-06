from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument("robot_name", default_value="r0", description="Robot namespace."),
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
        "lidar_topic",
        default_value=["/", LaunchConfiguration("robot_name"), "/sensors/lidar3d_0/points"],
        description="Input PointCloud2 topic for FAST_LIO.",
    ),
    DeclareLaunchArgument(
        "imu_topic",
        default_value=["/", LaunchConfiguration("robot_name"), "/sensors/imu_0/data"],
        description="Input Imu topic for FAST_LIO.",
    ),
    DeclareLaunchArgument("scan_line", default_value="16", description="LiDAR vertical channels."),
    DeclareLaunchArgument("scan_rate", default_value="50", description="LiDAR scan rate [Hz]."),
    DeclareLaunchArgument(
        "raw_odom_topic",
        default_value=["/", LaunchConfiguration("robot_name"), "/fastlio/raw_odom"],
        description="Internal FAST_LIO odometry topic before frame adaptation.",
    ),
    DeclareLaunchArgument(
        "odom_topic",
        default_value=["/", LaunchConfiguration("robot_name"), "/fastlio/odom"],
        description="Adapted odometry topic exposed to the rest of the stack.",
    ),
    DeclareLaunchArgument(
        "odom_frame_id",
        default_value=[LaunchConfiguration("robot_name"), "/odom"],
        description="Frame id used by the adapted odometry.",
    ),
    DeclareLaunchArgument(
        "base_frame_id",
        default_value=[LaunchConfiguration("robot_name"), "/base_link"],
        description="Child frame id used by the adapted odometry.",
    ),
    DeclareLaunchArgument(
        "fastlio_body_frame_id",
        default_value=[LaunchConfiguration("robot_name"), "/imu_0_link"],
        description="Robot TF frame corresponding to FAST_LIO's internal body frame.",
    ),
    DeclareLaunchArgument(
        "fastlio_internal_tf_topic",
        default_value=["/", LaunchConfiguration("robot_name"), "/fastlio/internal_tf"],
        description="Private TF topic for FAST_LIO's native camera_init->body transform.",
    ),
    DeclareLaunchArgument(
        "fastlio_internal_tf_static_topic",
        default_value=["/", LaunchConfiguration("robot_name"), "/fastlio/internal_tf_static"],
        description="Private TF static topic for FAST_LIO internals.",
    ),
    DeclareLaunchArgument(
        "broadcast_tf",
        default_value="true",
        choices=["true", "false"],
        description="Broadcast odom_frame_id -> base_frame_id from the adapted odometry.",
    ),
    DeclareLaunchArgument(
        "use_body_to_base_tf",
        default_value="true",
        choices=["true", "false"],
        description="Apply static robot TF from the FAST_LIO body frame to base_frame_id.",
    ),
    DeclareLaunchArgument(
        "lookup_timeout_sec",
        default_value="0.05",
        description="TF lookup timeout for body->base correction.",
    ),
    DeclareLaunchArgument("log_level", default_value="info", description="ROS log level."),
]


def generate_launch_description() -> LaunchDescription:
    config_file_path = PathJoinSubstitution([
        LaunchConfiguration("config_path"),
        LaunchConfiguration("config_file"),
    ])

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            config_file_path,
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "common.lid_topic": LaunchConfiguration("lidar_topic"),
                "common.imu_topic": LaunchConfiguration("imu_topic"),
                "preprocess.scan_line": ParameterValue(LaunchConfiguration("scan_line"), value_type=int),
                "preprocess.scan_rate": ParameterValue(LaunchConfiguration("scan_rate"), value_type=int),
            },
        ],
        remappings=[
            ("/Odometry", LaunchConfiguration("raw_odom_topic")),
            ("/cloud_registered", ["/", LaunchConfiguration("robot_name"), "/fastlio/cloud_registered"]),
            ("/cloud_registered_body", ["/", LaunchConfiguration("robot_name"), "/fastlio/cloud_registered_body"]),
            ("/cloud_effected", ["/", LaunchConfiguration("robot_name"), "/fastlio/cloud_effected"]),
            ("/Laser_map", ["/", LaunchConfiguration("robot_name"), "/fastlio/laser_map"]),
            ("/path", ["/", LaunchConfiguration("robot_name"), "/fastlio/path"]),
            ("/tf", LaunchConfiguration("fastlio_internal_tf_topic")),
            ("/tf_static", LaunchConfiguration("fastlio_internal_tf_static_topic")),
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    odom_adapter_node = Node(
        package="slam_evaluation",
        executable="fastlio_odom_adapter",
        name="fastlio_odom_adapter",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "input_odom_topic": LaunchConfiguration("raw_odom_topic"),
                "output_odom_topic": LaunchConfiguration("odom_topic"),
                "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "fastlio_body_frame_id": LaunchConfiguration("fastlio_body_frame_id"),
                "broadcast_tf": ParameterValue(LaunchConfiguration("broadcast_tf"), value_type=bool),
                "use_body_to_base_tf": ParameterValue(
                    LaunchConfiguration("use_body_to_base_tf"),
                    value_type=bool,
                ),
                "lookup_timeout_sec": ParameterValue(
                    LaunchConfiguration("lookup_timeout_sec"),
                    value_type=float,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    launch_description = LaunchDescription(ARGUMENTS)
    launch_description.add_action(fast_lio_node)
    launch_description.add_action(odom_adapter_node)
    return launch_description
