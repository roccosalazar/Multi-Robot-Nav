import os
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


PARAM_MAPPING: dict[str, type] = {
    "model_namespace": str,
    "tf_topics_in_model_namespace": bool,
    "prefix_frames_with_model_namespace": bool,
    "use_sim_time": bool,
    "enable_prefiltering": bool,
    "enable_scan_matching_odometry": bool,
    "enable_floor_detection": bool,
    "enable_gps": bool,
    "enable_imu_acceleration": bool,
    "enable_imu_orientation": bool,
    "enable_mrg_slam": bool,
    "enable_map2odom_publisher": bool,
    "tf_link_values": str,
    "points_topic": str,
    "map_frame_id": str,
    "odom_frame_id": str,
    "robot_odom_frame_id": str,
    "enable_robot_odometry_init_guess": bool,
    "imu_topic": str,
    "x": float,
    "y": float,
    "z": float,
    "roll": float,
    "pitch": float,
    "yaw": float,
    "init_odom_topic": str,
    "init_pose_topic": str,
    "result_dir": str,
    "registration_method": str,
}


Remapping = Tuple[str, str]
ParameterDict = Dict[str, Any]


def parse_bool(value: str) -> bool:
    """
    Convert a CLI string into a boolean.

    Accepted true values: true, 1, yes, on.
    Accepted false values: false, 0, no, off.

    Args:
        value: String value coming from ROS 2 launch CLI arguments.

    Returns:
        Parsed boolean value.

    Raises:
        ValueError: If the input cannot be interpreted as a boolean.
    """
    normalized: str = value.strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"Invalid boolean value: '{value}'")


def parse_cli_value(param_name: str, raw_value: str) -> Any:
    """
    Convert a CLI string to the expected parameter type.

    Args:
        param_name: Parameter name used to retrieve the expected type.
        raw_value: Raw string value from launch configurations.

    Returns:
        Parsed value with the expected Python type.

    Raises:
        KeyError: If the parameter is not present in PARAM_MAPPING.
        ValueError: If conversion fails.
    """
    expected_type: type = PARAM_MAPPING[param_name]

    if expected_type is bool:
        return parse_bool(raw_value)

    return expected_type(raw_value)


def overwrite_yaml_params_from_cli(
    yaml_params: ParameterDict,
    cli_params: dict[str, str],
    excluded_keys: Optional[Iterable[str]] = None,
) -> ParameterDict:
    """
    Override YAML parameters with non-empty CLI values.

    Args:
        yaml_params: Parameters loaded from YAML.
        cli_params: Launch configuration key/value pairs.
        excluded_keys: Optional parameter names that must not be overridden.

    Returns:
        Updated parameter dictionary.
    """
    excluded: set[str] = set(excluded_keys or [])

    for key, raw_value in cli_params.items():
        if key in excluded:
            continue
        if key not in yaml_params:
            continue
        if raw_value == "":
            continue
        if key not in PARAM_MAPPING:
            continue

        yaml_params[key] = parse_cli_value(key, raw_value)

    return yaml_params


def print_yaml_params(yaml_params: ParameterDict, header: Optional[str] = None) -> None:
    """
    Print parameters in YAML format for debugging.

    Args:
        yaml_params: Parameter dictionary to print.
        header: Optional section header.
    """
    if header:
        print(f"######## {header} ########")
    print(yaml.dump(yaml_params, sort_keys=False, default_flow_style=False))


def print_remappings(remappings: Sequence[Remapping], header: Optional[str] = None) -> None:
    """
    Print a list of remappings for debugging.

    Args:
        remappings: Sequence of (source, target) remappings.
        header: Optional section header.
    """
    if header:
        print(f"######## {header} remappings ########")
    for source, target in remappings:
        print(f"{source} -> {target}")
    print("")


def prefixed_frame(frame_id: str, model_namespace: str, prefix_enabled: bool) -> str:
    """
    Prefix a frame id with the model namespace when required.

    Args:
        frame_id: Original frame id.
        model_namespace: Robot namespace.
        prefix_enabled: Whether frame prefixing is enabled.

    Returns:
        Prefixed or original frame id.
    """
    if model_namespace and prefix_enabled:
        return f"{model_namespace}/{frame_id}"
    return frame_id


def build_static_transform_publisher(
    *,
    name: str,
    namespace: str,
    params: ParameterDict,
    shared_params: ParameterDict,
    tf_remappings: Sequence[Remapping],
    frame_id: str,
    child_frame_id: str,
) -> Node:
    """
    Create a tf2 static transform publisher node.

    Args:
        name: Node name.
        namespace: ROS namespace.
        params: Transform parameters dictionary.
        shared_params: Shared ROS parameters.
        tf_remappings: TF remappings.
        frame_id: Parent frame id.
        child_frame_id: Child frame id.

    Returns:
        Configured Node action.
    """
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        namespace=namespace,
        output="both",
        parameters=[shared_params],
        remappings=list(tf_remappings),
        arguments=[
            "--x", str(params["x"]),
            "--y", str(params["y"]),
            "--z", str(params["z"]),
            "--roll", str(params["roll"]),
            "--pitch", str(params["pitch"]),
            "--yaw", str(params["yaw"]),
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
        ],
    )


def build_composable_node(
    *,
    package: str,
    plugin: str,
    name: str,
    namespace: str,
    parameters: Sequence[ParameterDict],
    remappings: Sequence[Remapping],
) -> ComposableNode:
    """
    Create a composable node with intra-process communications enabled.

    Args:
        package: ROS package name.
        plugin: Fully qualified plugin class name.
        name: Node name.
        namespace: ROS namespace.
        parameters: Node parameter dictionaries.
        remappings: Topic remappings.

    Returns:
        Configured ComposableNode description.
    """
    return ComposableNode(
        package=package,
        plugin=plugin,
        name=name,
        namespace=namespace,
        parameters=list(parameters),
        remappings=list(remappings),
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def load_config(config_file_name: str) -> dict[str, Any]:
    """
    Load the mrg_slam YAML configuration file.

    Args:
        config_file_name: YAML file name inside the package config directory.

    Returns:
        Parsed YAML configuration.

    Raises:
        FileNotFoundError: If the config file does not exist.
        KeyError: If expected YAML sections are missing.
    """
    config_file_path: str = os.path.join(
        get_package_share_directory("mrg_slam"),
        "config",
        config_file_name,
    )

    with open(config_file_path, "r", encoding="utf-8") as file:
        config: dict[str, Any] = yaml.safe_load(file)

    print(f"Loaded config file: {config_file_path}")
    return config


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> List[Any]:
    """
    Build the launch actions using runtime launch arguments.

    Args:
        context: Launch runtime context.
        *args: Unused extra positional arguments.
        **kwargs: Unused extra keyword arguments.

    Returns:
        List of launch actions to be returned by OpaqueFunction.
    """
    del args
    del kwargs

    launch_configurations: dict[str, str] = context.launch_configurations
    config_file_name: str = launch_configurations.get("config", "mrg_slam.yaml")
    config: dict[str, Any] = load_config(config_file_name)

    shared_params: ParameterDict = config["/**"]["ros__parameters"]
    lidar2base_publisher_params: ParameterDict = config["lidar2base_publisher"]["ros__parameters"]
    map2robotmap_publisher_params: ParameterDict = config["map2robotmap_publisher"]["ros__parameters"]
    prefiltering_params: ParameterDict = config["prefiltering_component"]["ros__parameters"]
    scan_matching_odometry_params: ParameterDict = config["scan_matching_odometry_component"]["ros__parameters"]
    floor_detection_params: ParameterDict = config["floor_detection_component"]["ros__parameters"]
    mrg_slam_params: ParameterDict = config["mrg_slam_component"]["ros__parameters"]

    shared_params = overwrite_yaml_params_from_cli(shared_params, launch_configurations)
    lidar2base_publisher_params = overwrite_yaml_params_from_cli(lidar2base_publisher_params, launch_configurations)
    map2robotmap_publisher_params = overwrite_yaml_params_from_cli(
        map2robotmap_publisher_params,
        launch_configurations,
        excluded_keys={"x", "y", "z", "roll", "pitch", "yaw"},
    )
    prefiltering_params = overwrite_yaml_params_from_cli(prefiltering_params, launch_configurations)
    scan_matching_odometry_params = overwrite_yaml_params_from_cli(scan_matching_odometry_params, launch_configurations)
    floor_detection_params = overwrite_yaml_params_from_cli(floor_detection_params, launch_configurations)
    mrg_slam_params = overwrite_yaml_params_from_cli(mrg_slam_params, launch_configurations)

    model_namespace: str = shared_params["model_namespace"]
    tf_topics_in_model_namespace: bool = shared_params.get("tf_topics_in_model_namespace", False)
    prefix_frames_with_model_namespace: bool = shared_params.get("prefix_frames_with_model_namespace", True)

    print_yaml_params(shared_params, "shared_params")
    print_yaml_params(lidar2base_publisher_params, "lidar2base_publisher_params")
    print_yaml_params(map2robotmap_publisher_params, "map2robotmap_publisher_params")
    print_yaml_params(prefiltering_params, "prefiltering_params")
    print_yaml_params(scan_matching_odometry_params, "scan_matching_odometry_params")
    print_yaml_params(floor_detection_params, "floor_detection_params")
    print_yaml_params(mrg_slam_params, "mrg_slam_params")

    tf_remappings: List[Remapping] = []
    if tf_topics_in_model_namespace:
        tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    print_remappings(tf_remappings, "tf_remappings")

    actions: List[Any] = []

    if lidar2base_publisher_params["enable_lidar2base_publisher"]:
        lidar_frame_id: str = prefixed_frame(
            lidar2base_publisher_params["frame_id"],
            model_namespace,
            prefix_frames_with_model_namespace,
        )
        lidar_child_frame_id: str = prefixed_frame(
            lidar2base_publisher_params["child_frame_id"],
            model_namespace,
            prefix_frames_with_model_namespace,
        )
        actions.append(
            build_static_transform_publisher(
                name="lidar2base_publisher",
                namespace=model_namespace,
                params=lidar2base_publisher_params,
                shared_params=shared_params,
                tf_remappings=tf_remappings,
                frame_id=lidar_frame_id,
                child_frame_id=lidar_child_frame_id,
            )
        )

    enable_map2robotmap_publisher: bool = (
        map2robotmap_publisher_params["enable_map2robotmap_publisher"]
        and bool(model_namespace)
        and prefix_frames_with_model_namespace
    )

    if (
        map2robotmap_publisher_params["enable_map2robotmap_publisher"]
        and model_namespace
        and not prefix_frames_with_model_namespace
    ):
        print(
            "Disabled map2robotmap_publisher because "
            "prefix_frames_with_model_namespace=false would create an invalid map->map transform."
        )

    if enable_map2robotmap_publisher:
        map2robotmap_child_frame_id: str = prefixed_frame(
            map2robotmap_publisher_params["child_frame_id"],
            model_namespace,
            prefix_frames_with_model_namespace,
        )
        actions.append(
            build_static_transform_publisher(
                name="map2robotmap_publisher",
                namespace=model_namespace,
                params=map2robotmap_publisher_params,
                shared_params=shared_params,
                tf_remappings=tf_remappings,
                frame_id=map2robotmap_publisher_params["frame_id"],
                child_frame_id=map2robotmap_child_frame_id,
            )
        )

    enable_map2odom_publisher: bool = bool(
        shared_params.get(
            "enable_map2odom_publisher",
            mrg_slam_params.get("enable_map2odom_publisher", True),
        )
    )
    raw_enable_map2odom_publisher: str = launch_configurations.get("enable_map2odom_publisher", "")
    if raw_enable_map2odom_publisher != "":
        enable_map2odom_publisher = parse_cli_value("enable_map2odom_publisher", raw_enable_map2odom_publisher)

    if enable_map2odom_publisher:
        actions.append(
            Node(
                package="mrg_slam",
                executable="map2odom_publisher_ros2.py",
                name="map2odom_publisher_ros2",
                namespace=model_namespace,
                output="both",
                parameters=[mrg_slam_params, shared_params],
                remappings=tf_remappings,
            )
        )

    container_prefix: List[str] = ["gdbserver localhost:3000"] if "debug" in launch_configurations else []
    container_name: str = f"{model_namespace}/mrg_slam_container" if model_namespace else "mrg_slam_container"

    container_remaps: List[Remapping] = list(tf_remappings) + [
        ("imu/data", shared_params["imu_topic"]),
        ("velodyne_points", shared_params["points_topic"]),
    ]
    print_remappings(container_remaps, "container_remappings")

    actions.append(
        Node(
            package="rclcpp_components",
            executable="component_container_mt",
            name="mrg_slam_container",
            namespace=model_namespace,
            output="both",
            parameters=[shared_params],
            prefix=container_prefix,
            remappings=container_remaps,
        )
    )

    composable_nodes: List[ComposableNode] = []

    if model_namespace and prefix_frames_with_model_namespace:
        prefiltering_params["base_link_frame"] = f"{model_namespace}/{prefiltering_params['base_link_frame']}"

    if prefiltering_params["enable_prefiltering"]:
        composable_nodes.append(
            build_composable_node(
                package="mrg_slam",
                plugin="mrg_slam::PrefilteringComponent",
                name="prefiltering_component",
                namespace=model_namespace,
                parameters=[prefiltering_params, shared_params],
                remappings=list(tf_remappings) + [
                    ("imu/data", shared_params["imu_topic"]),
                    ("velodyne_points", shared_params["points_topic"]),
                ],
            )
        )

    if model_namespace and prefix_frames_with_model_namespace:
        scan_matching_odometry_params["odom_frame_id"] = f"{model_namespace}/{scan_matching_odometry_params['odom_frame_id']}"
        scan_matching_odometry_params["robot_odom_frame_id"] = (
            f"{model_namespace}/{scan_matching_odometry_params['robot_odom_frame_id']}"
        )

    if scan_matching_odometry_params["enable_scan_matching_odometry"]:
        composable_nodes.append(
            build_composable_node(
                package="mrg_slam",
                plugin="mrg_slam::ScanMatchingOdometryComponent",
                name="scan_matching_odometry_component",
                namespace=model_namespace,
                parameters=[scan_matching_odometry_params, shared_params],
                remappings=tf_remappings,
            )
        )

        if scan_matching_odometry_params["enable_odom_to_file"]:
            result_file_suffix: str = model_namespace if model_namespace else "default"
            actions.append(
                Node(
                    package="mrg_slam",
                    executable="odom_to_file.py",
                    name="odom_to_file",
                    namespace=model_namespace,
                    output="screen",
                    remappings=[("odom", "scan_matching_odometry/odom")],
                    parameters=[{
                        "result_file": f"/tmp/{result_file_suffix}_scan_matching_odom.txt",
                        "every_n": 1,
                    }],
                )
            )

    if floor_detection_params["enable_floor_detection"]:
        composable_nodes.append(
            build_composable_node(
                package="mrg_slam",
                plugin="mrg_slam::FloorDetectionComponent",
                name="floor_detection_component",
                namespace=model_namespace,
                parameters=[floor_detection_params, shared_params],
                remappings=tf_remappings,
            )
        )

    if mrg_slam_params["enable_mrg_slam"]:
        if model_namespace:
            mrg_slam_params["own_name"] = model_namespace

        init_pose: list[float] = list(mrg_slam_params["init_pose"])
        init_pose[0] = mrg_slam_params["x"]
        init_pose[1] = mrg_slam_params["y"]
        init_pose[2] = mrg_slam_params["z"]
        init_pose[3] = mrg_slam_params["roll"]
        init_pose[4] = mrg_slam_params["pitch"]
        init_pose[5] = mrg_slam_params["yaw"]
        mrg_slam_params["init_pose"] = init_pose

        if model_namespace and prefix_frames_with_model_namespace:
            mrg_slam_params["map_frame_id"] = f"{model_namespace}/{mrg_slam_params['map_frame_id']}"
            mrg_slam_params["odom_frame_id"] = f"{model_namespace}/{mrg_slam_params['odom_frame_id']}"

        composable_nodes.append(
            build_composable_node(
                package="mrg_slam",
                plugin="mrg_slam::MrgSlamComponent",
                name="mrg_slam_component",
                namespace=model_namespace,
                parameters=[mrg_slam_params, shared_params],
                remappings=list(tf_remappings) + [("imu/data", shared_params["imu_topic"])],
            )
        )

    actions.append(
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=composable_nodes,
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Generate the ROS 2 launch description.

    All parameters declared in PARAM_MAPPING can be overridden from CLI.
    The actual launch graph is built at runtime through OpaqueFunction.

    Returns:
        LaunchDescription instance.
    """
    declared_arguments: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(name=param_name, default_value="")
        for param_name in PARAM_MAPPING
    ]

    declared_arguments.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(declared_arguments)