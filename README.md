# Multi-Robot-Nav

Multi-Robot-Nav is a ROS 2 simulation and orchestration repository for multi-robot LiDAR SLAM experiments with Clearpath Warthog-class robots.

This codebase combines three layers:

1. Clearpath simulation and robot-generation tooling (`clearpath_ws` + per-robot setup folders in `robots`).
2. The Multi-Robot Graph SLAM stack (`Multi-Robot-Graph-SLAM`, including `mrg_slam`).
3. A custom project workspace (`workspace`) that provides project-specific bringup and SLAM evaluation nodes.

The current custom logic in this repository is centered on:

- launching one/two/three simulated robots with reproducible spawn poses,
- launching single-robot and multi-robot `mrg_slam` instances in robot namespaces,
- publishing comparable pose streams for evaluation:
  - ground truth from Gazebo dynamic pose,
  - SLAM pose from TF (`map -> <robot>/base_link`) published on `/<robot>/slam/pose` with `header.frame_id=<world>` (default `warehouse`).

## What The Project Is Building

The project develops a multi-robot system based on three named robots:

- `aramis`
- `athos`
- `porthos`

Each robot is configured as a Clearpath Warthog (`w200`) with tracked base and a 3D Velodyne LiDAR in its robot setup folder under `robots/<name>`.

The intended system architecture is:

- simulation world in Gazebo (via `clearpath_gz`),
- one robot namespace per platform (`/aramis`, `/athos`, `/porthos`),
- one `mrg_slam` instance per robot namespace,
- shared/global TF transport with namespaced frame IDs enabled by `mrg_slam` config,
- optional extraction of ground-truth and SLAM pose topics for evaluation.

## Repository Architecture

Top-level directories:

- `workspace/`: project-specific ROS 2 workspace (bringup + evaluation code).
- `robots/`: generated Clearpath setup for each robot (`aramis`, `athos`, `porthos`).
- `clearpath_ws/`: vendored Clearpath ROS 2 stack used by spawning and simulation.
- `Multi-Robot-Graph-SLAM/`: vendored SLAM stack containing `mrg_slam` and dependencies.
- `.devcontainer/`: development container definitions for AMD64 and ARM64 environments.
- `debug/reports/utils/`: local URDF/Xacro and RViz prototyping assets used for payload and model iteration.
- `source_workspaces.sh`: helper script that sources all three local workspaces.

## How Clearpath Files Fit Into This Project

Clearpath integration happens through two pieces:

1. Clearpath packages in `clearpath_ws/src`:
	- `clearpath_gz` provides Gazebo launch entry points (`gz_sim.launch.py`, `robot_spawn.launch.py`).
	- `clearpath_generator_common` and `clearpath_generator_gz` generate robot-specific launch/config artifacts.
	- `clearpath_common`, `clearpath_control`, `clearpath_description`, and related description packages provide robot model, control, localization, and teleop launch components.
	- `clearpath_config` parses each `robot.yaml` to extract namespace/system configuration.

2. Robot setup folders in `robots/`:
	- `robots/aramis`, `robots/athos`, `robots/porthos` each contain:
	  - `robot.yaml` (namespace and platform/sensor config),
	  - generated `platform/launch` and `sensors/launch` files,
	  - generated sensor bridge configs (`sensors/config/*.yaml`),
	  - generated platform configs (`platform/config/*.yaml`),
	  - robot description files (`robot.urdf.xacro`, `robot.srdf*`).

In this repository, `musketeers_bringup/spawn_robot.launch.py` passes
`setup_path=$HOME/Multi-Robot-Nav/robots/<robot_name>` to Clearpath's `robot_spawn.launch.py`.
That is the key handoff from custom bringup to Clearpath-generated robot assets.

## How `mrg_slam` Is Used Here

`mrg_slam` is consumed from `Multi-Robot-Graph-SLAM/src/mrg_slam` and launched by
`workspace/src/musketeers_bringup/launch/slam_bringup.launch.py`.

`slam_bringup.launch.py` does two things for one robot namespace:

1. Includes `mrg_slam/launch/mrg_slam.launch.py` with arguments:
	- `config` (default `aramis.yaml` from `mrg_slam/config`),
	- `model_namespace` = `robot_name`,
	- `use_sim_time`,
	- initial `x/y/z`.
2. Starts `slam_evaluation/slam_pose_publisher` to publish `/<robot>/slam/pose` from TF (`map -> <robot>/base_link`) and sets output `header.frame_id` from `world` (default `warehouse`).

Relevant `mrg_slam` config files present in this repository:

- `mrg_slam/config/aramis.yaml`: single-robot namespace defaults.
- `mrg_slam/config/musketeers.yaml`: multi-robot-oriented shared settings (`multi_robot_names: ["athos", "porthos", "aramis"]`).

`multi_slam_bringup.launch.py` starts three SLAM instances (`aramis`, `athos`, `porthos`) in one command. `slam_bringup.launch.py` remains the single-robot entry point.

## Project-Specific Changes To Vendored Upstreams

This repository contains local modifications on top of the imported `clearpath_ws` and `Multi-Robot-Graph-SLAM` sources.

### Version-control layout used in this project

- `clearpath_ws` and `Multi-Robot-Graph-SLAM` are vendored into this monorepo.
- Nested `.git` directories from the original upstream repositories are not used.
- All project changes are tracked in the root repository history.

### Changes made in `clearpath_ws`

The local Clearpath stack was modified mainly to normalize TF behavior and improve multi-robot simulation integration.

1. TF remapping normalized to global topics
	- Multiple launch and generator files were changed from relative remaps (`/tf -> tf`, `/tf_static -> tf_static`) to absolute/global remaps (`/tf -> /tf`, `/tf_static -> /tf_static`).
	- Modified files include:
	  - `clearpath_control/launch/control.launch.py`
	  - `clearpath_control/launch/localization.launch.py`
	  - `clearpath_control/launch/teleop_base.launch.py`
	  - `clearpath_control/launch/teleop_joy.launch.py`
	  - `clearpath_generator_common/common.py`
	  - `clearpath_platform_description/launch/description.launch.py`
	  - `clearpath_manipulators_description/launch/description.launch.py`
	  - `clearpath_platform_description/urdf/generic/gazebo.urdf.xacro`

2. Robot frame prefixing and namespace consistency
	- `robot_state_publisher` launch paths now compute and pass a `frame_prefix` from the active namespace.
	- Sensor launch generation was adjusted so generated static TF parent links are namespace-prefixed when needed.

3. Gazebo-to-ROS TF bridge routing changed
	- In `clearpath_generator_gz/launch/generator.py`, bridge remapping for model TF was changed to publish into global `/tf`.
	- IMU filter remapping to `/tf` was also normalized.

4. Warthog control TF behavior changed
	- In `clearpath_control/config/w200/control.yaml`:
	  - `enable_odom_tf` changed to `true`.
	  - `tf_frame_prefix_enable` changed to `true`.

5. Generated launch behavior changed for W200
	- In `clearpath_generator_gz/launch/generator.py`, the W200 launch sequence removed `odom_base_node` from the generated platform action list.

6. Sensor frame naming alignment
	- In `clearpath_sensors_description/urdf/velodyne_lidar.urdf.xacro`, `ignition_frame_id` is now namespace-aware (`<namespace>/<sensor>_laser` when namespace is set).

7. Simulation world tuning
	- `clearpath_gz/worlds/pipeline.sdf` and `clearpath_gz/worlds/warehouse.sdf` were locally tuned:
	  - `max_step_size` changed to `0.01`.
	  - DART collision detector set to `bullet`.
	  - Additional scene/plugin adjustments in `pipeline.sdf`.

8. Repository-level cleanup/artifacts in `clearpath_ws`
	- `clearpath_ws/dependencies.repos` removed.
	- Benchmark/report artifact logs added under `clearpath_ws/reports/artifacts/`.

9. W200 body color parameter support added
	- In `clearpath_config/platform/platform.py`, optional `platform.color` was added (default `yellow`).
	- In `clearpath_generator_common/description/platform.py`, W200 description generation now forwards `color` to the `<xacro:w200 ...>` macro.
	- This allows selecting body color from each robot setup `robot.yaml` without manually editing generated URDF/Xacro files.
	- Supported W200 color values are `yellow`, `orange`, `olive`, and `sand`.

### Changes made in `Multi-Robot-Graph-SLAM`

The local SLAM stack was modified to integrate with the Clearpath topic/frame conventions above.

1. TF topic strategy made configurable in `mrg_slam.launch.py`
	- Added launch parameter support for:
	  - `tf_topics_in_model_namespace`
	  - `prefix_frames_with_model_namespace`
	- TF remappings are now conditional:
	  - if `tf_topics_in_model_namespace=true`, nodes use namespaced `tf`/`tf_static`.
	  - if `false`, nodes use global `/tf` and `/tf_static`.

2. Frame prefix logic generalized in `mrg_slam.launch.py`
	- Added `maybe_prefix_frame(...)` handling.
	- Prefixed frame IDs are now applied conditionally instead of unconditionally.
	- `map2robotmap_publisher` is guarded to avoid invalid `map -> map` transforms when frame prefixing is disabled.

3. Component remappings aligned with TF strategy
	- `tf_remappings` are now propagated to prefiltering, floor detection, and SLAM component remaps where needed.

4. Config updates for Clearpath topics and TF policy
	- In `mrg_slam/config/aramis.yaml`, `mrg_slam/config/mrg_slam.yaml`, and `mrg_slam/config/musketeers.yaml`:
	  - `tf_topics_in_model_namespace: false`
	  - `prefix_frames_with_model_namespace: true`
	  - LiDAR topic switched to `sensors/lidar3d_0/points` for Clearpath integration.
	  - IMU topic aligned to `sensors/imu_0/data` where applicable.
	  - Sensor frame naming updated to `lidar3d_0_laser` in robot-specific config.

5. Prefiltering robustness patch
	- In `mrg_slam/apps/prefiltering_component.cpp`, TF lookup now has a fallback path that prepends namespace to source frame when incoming PointCloud2 frame IDs are unprefixed but TF tree is prefixed.

6. Launch parameter overwrite hardening
	- In `mrg_slam/launch/mrg_slam.launch.py`, `overwrite_yaml_params_from_cli(...)` now supports `excluded_keys`.
	- For `map2robotmap_publisher`, CLI pose arguments (`x/y/z/roll/pitch/yaw`) are excluded so the static `map -> <robot>/map` transform remains driven by YAML values.

### Changes made in local robot and tooling layers

These changes are project-local (outside `clearpath_ws` and `Multi-Robot-Graph-SLAM`) and support simulation realism, prototyping, and developer workflow.

1. Payload physical model and control updates
	- In `robots/payloads/canisters.xacro`, multiple payload links now include inertial and collision properties for more realistic dynamics.
	- Canister joints were tuned (axis/limits/effort/dynamics) and simulation control interfaces were added.
	- Gazebo joint position controllers expose namespaced command topics:
	  - `/<namespace>/left_canister/cmd_pos`
	  - `/<namespace>/right_canister/cmd_pos`

2. Local URDF/Xacro prototyping assets
	- Added experimental IMPACT turret and integration assets under `debug/reports/utils/canister/`.
	- Additional prototyping files are present in `debug/reports/utils/` and `debug/reports/utils/canister_2/` for payload iteration.

3. Devcontainer tooling update
	- Added `ripgrep` to both `.devcontainer/Dockerfile.amd64` and `.devcontainer/Dockerfile.arm64` for faster in-container code search.

### Net effect for this project

These local upstream changes support the project architecture in this repository:

- global/shared TF transport (`/tf`, `/tf_static`) across all robots,
- namespaced frame IDs to keep robot frames distinct,
- direct consumption of Clearpath sensor topics by `mrg_slam`,
- more reliable frame resolution in mixed prefixed/unprefixed simulation conditions,
- namespaced simulation control of canister joints,
- updated RViz layouts for both single-robot and three-robot SLAM monitoring.

## Workspace Directory (Complete Guide)

This section documents the full role of the `workspace/` directory.

### `workspace/` top level

- `workspace/src/`: source packages with project logic.
- `workspace/build/`: colcon build artifacts (contains `musketeers_bringup/` and `slam_evaluation/` build outputs).
- `workspace/install/`: install space and setup scripts (`setup.bash`, `local_setup.bash`, etc.) plus installed package shares.
- `workspace/log/`: colcon build logs (timestamped `build_*` directories and `latest` links).

### `workspace/src/` package structure

```text
workspace/src/
├── musketeers_bringup/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   ├── spawn_world.launch.py
│   │   ├── spawn_robot.launch.py
│   │   ├── spawn_musketeers.launch.py
│   │   ├── slam_bringup.launch.py
│   │   └── multi_slam_bringup.launch.py
│   └── rviz/
│       ├── aramis.rviz
│       └── multi_slam.rviz
└── slam_evaluation/
	 ├── package.xml
	 ├── setup.py
	 ├── setup.cfg
	 ├── resource/slam_evaluation
	 ├── launch/
	 │   ├── ground_truth_pose.launch.py
	 │   └── slam_pose_publisher.launch.py
	 ├── slam_evaluation/
	 │   ├── __init__.py
	 │   ├── ground_truth_pose_extractor.py
	 │   └── slam_pose_publisher.py
	 └── test/
		  ├── test_copyright.py
		  ├── test_flake8.py
		  └── test_pep257.py
```

## Package Responsibilities In `workspace/src`

### `musketeers_bringup` (bringup/orchestration)

Type: `ament_cmake`

Purpose:

- launch orchestration for world and robot spawning,
- staged multi-robot spawn scenarios,
- single-robot and multi-robot SLAM startup wrappers.

Dependencies (from `package.xml`):

- `clearpath_gz`
- `mrg_slam`
- `slam_evaluation`
- ROS 2 launch packages

Important launch files:

1. `spawn_world.launch.py`
	- Includes Clearpath `clearpath_gz/launch/gz_sim.launch.py`.
	- Argument: `world` (default `warehouse`).
	- Sets NVIDIA offload environment variables when NVIDIA devices are present.
	- Category: simulation bringup.

2. `spawn_robot.launch.py`
	- Includes Clearpath `clearpath_gz/launch/robot_spawn.launch.py`.
	- Uses robot setup path: `$HOME/Multi-Robot-Nav/robots/<robot_name>`.
	- Arguments include pose (`x/y/z/yaw`), `world`, `rviz`, `use_sim_time`, `generate`.
	- Optionally includes `slam_evaluation/launch/ground_truth_pose.launch.py` (`ground_truth_pose:=true` by default).
	- Category: robot bringup + simulation bridge + optional evaluation input.

3. `spawn_musketeers.launch.py`
	- Spawns `aramis`, `athos`, `porthos` with delays.
	- Fixed poses:
	  - aramis: `(-3.0, 0.0, 0.3, yaw 0.0)`
	  - athos: `(0.0, 0.0, 0.3, yaw 0.0)` after 7 s
	  - porthos: `(3.0, 0.0, 0.3, yaw 0.0)` after 14 s
	- Category: multi-robot simulation orchestration.

4. `slam_bringup.launch.py`
	- Includes `mrg_slam.launch.py` for one robot namespace.
	- Starts `slam_evaluation/slam_pose_publisher` in same namespace.
	- Publishes `slam/pose` from TF lookup (`map -> <namespace>/base_link`) with configurable xyz offset, output frame id (`world`) and lookup timeout.
	- Category: SLAM bringup + SLAM output normalization for evaluation.

5. `multi_slam_bringup.launch.py`
	- Includes `slam_bringup.launch.py` three times for `aramis`, `athos`, `porthos`.
	- Defaults to `config:=musketeers.yaml` and robot initial poses:
	  - aramis: `x:=-3.0 y:=0.0 z:=0.0`
	  - athos: `x:=0.0 y:=0.0 z:=0.0`
	  - porthos: `x:=3.0 y:=0.0 z:=0.0`
	- Category: multi-robot SLAM bringup + SLAM output normalization for evaluation.

6. `rviz/aramis.rviz`
	- Single-robot RViz profile for `aramis`, configured for namespaced topics such as `/aramis/mrg_slam/map_points`, `/aramis/mrg_slam/markers`, and `/aramis/scan_matching_odometry/odom`.
	- Includes current payload/canister TF/link visualization.
	- Category: SLAM visualization.

7. `rviz/multi_slam.rviz`
	- Multi-robot RViz profile covering `aramis`, `athos`, and `porthos` TF trees and robot descriptions.
	- Includes SLAM map/marker/odometry views for multi-robot runs.
	- Category: SLAM visualization.

### `slam_evaluation` (SLAM vs ground-truth pose extraction)

Type: `ament_python`

Purpose:

- convert Gazebo dynamic pose stream into per-robot `PoseStamped` ground truth,
- publish a per-robot `PoseStamped` SLAM estimate from TF,
- provide normalized topics for downstream metrics tooling.

Python nodes:

1. `ground_truth_pose_extractor.py`
	- Subscribes to `TFMessage` (`ground_truth/pose_raw`).
	- Selects transform whose child frame matches `<robot_name>/robot`.
	- Publishes `PoseStamped` on `ground_truth/pose`.
	- Fallback behavior if incoming TF stamp or frame is empty.

2. `slam_pose_publisher.py`
	- Looks up TF transform `map_frame -> target_base_frame` (default `map -> <namespace>/base_link`).
	- Publishes `PoseStamped` on `slam/pose`.
	- Supports configurable publish rate, lookup timeout, xyz offsets, and output `frame_id`.

Launch files:

1. `ground_truth_pose.launch.py`
	- Starts `ros_gz_bridge parameter_bridge` from:
	  - `/world/<world>/dynamic_pose/info`
	  - to namespaced `ground_truth/pose_raw`.
	- Starts `ground_truth_pose_extractor`.
	- Default world argument is `warehouse`.

2. `slam_pose_publisher.launch.py`
	- Starts only `slam_pose_publisher` node with configurable lookup frames, output `frame_id`, rate, timeout, and offsets.

## Launch Connectivity: End-To-End Data Flow

### Simulation and robot interfaces

1. `spawn_world.launch.py` starts Gazebo world via Clearpath `gz_sim.launch.py`.
2. `spawn_robot.launch.py` (or multi-robot variants) calls Clearpath `robot_spawn.launch.py`.
3. Clearpath launch chain uses robot setup folder (`robots/<name>`) to start:
	- platform bringup (`platform-service.launch.py`),
	- sensors bringup (`sensors-service.launch.py`),
	- Gazebo bridge nodes (e.g., LiDAR/IMU, cmd_vel),
	- robot entity creation in Gazebo.

### SLAM and evaluation

1. `slam_bringup.launch.py` launches one `mrg_slam` instance in `/<robot_name>` namespace.
2. `mrg_slam` consumes Clearpath LiDAR points (`sensors/lidar3d_0/points` per config) and publishes SLAM TF/map outputs.
3. `slam_pose_publisher` converts SLAM TF to `/<robot_name>/slam/pose` using TF `map -> <robot>/base_link` and publishes with `header.frame_id=<world>`.
4. If enabled in `spawn_robot.launch.py`, `ground_truth_pose.launch.py` publishes `/<robot_name>/ground_truth/pose` from Gazebo dynamic pose.

This gives two comparable pose streams per robot:

- `/<robot>/slam/pose`
- `/<robot>/ground_truth/pose`

## Simulation, SLAM, Navigation, Robot, And Bringup Boundaries

- Simulation-related:
  - `clearpath_gz` world and robot spawning,
  - Gazebo world files (`clearpath_gz/worlds/*.sdf`),
  - robot sensor and cmd_vel bridges from generated robot launch files.

- SLAM-related:
  - `mrg_slam` package and configs in `Multi-Robot-Graph-SLAM/src/mrg_slam`,
	- `slam_bringup.launch.py` and `multi_slam_bringup.launch.py`,
  - `slam_evaluation` nodes and launch files.

- Navigation-related (current state):
  - no custom Nav2 package exists in `workspace/src`,
  - robot motion/control path currently comes from Clearpath platform/control stack (controllers, localization, teleop, cmd_vel bridge),
  - this repository does not add a dedicated autonomous navigation planner layer in custom packages.

- Robot-related:
  - per-robot definitions in `robots/<name>/robot.yaml` and `robot.urdf.xacro`,
  - generated robot-specific platform/sensor launch and bridge config.

- Bringup-related:
  - orchestration package `musketeers_bringup`.

## Build And Environment Setup

Build order used in this repository:

```bash
cd clearpath_ws
colcon build --symlink-install

cd ../Multi-Robot-Graph-SLAM
colcon build --symlink-install

cd ../workspace
colcon build --symlink-install

cd ..
```

Source ROS 2 and local workspaces:

```bash
source /opt/ros/humble/setup.bash
source source_workspaces.sh
```

`source_workspaces.sh` currently sources:

```bash
source ./workspace/install/setup.bash
source ./clearpath_ws/install/setup.bash
source ./Multi-Robot-Graph-SLAM/install/setup.bash
```

## Typical Launch Sequences

### A) World only

```bash
ros2 launch musketeers_bringup spawn_world.launch.py world:=warehouse
```

### B) Single robot + optional ground truth

```bash
ros2 launch musketeers_bringup spawn_robot.launch.py \
  robot_name:=aramis \
  x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 \
  world:=warehouse \
  ground_truth_pose:=true
```

### C) Three robots

```bash
ros2 launch musketeers_bringup spawn_musketeers.launch.py world:=warehouse
```

### D) Single robot SLAM instance

```bash
ros2 launch musketeers_bringup slam_bringup.launch.py \
  robot_name:=aramis \
  config:=aramis.yaml \
  use_sim_time:=true \
	world:=warehouse \
	x:=0.0 y:=0.0 z:=0.0 \
	slam_pose_offset_x:=0.0 slam_pose_offset_y:=0.0 slam_pose_offset_z:=0.35
```

### E) Three-robot SLAM bringup

```bash
ros2 launch musketeers_bringup multi_slam_bringup.launch.py \
	use_sim_time:=true \
	world:=warehouse
```

### F) Multi-robot RViz profile

```bash
rviz2 -d $HOME/Multi-Robot-Nav/workspace/src/musketeers_bringup/rviz/multi_slam.rviz
```

`multi_slam_bringup.launch.py` defaults to `config:=musketeers.yaml`.

## Practical Notes

- `spawn_robot.launch.py` defaults `generate:=false`, so it expects robot-specific generated files to already exist under `robots/<name>/...`.
- Generated robot launch files currently contain absolute paths rooted at `/home/ubuntu/Multi-Robot-Nav/...`.
- World defaults are now aligned to `warehouse` across workspace launch files.
- W200 body color can now be configured per robot setup using `platform.color` (for example `olive` to match the IMPACT palette).
- Canister joint command topics are namespaced as `/<robot_namespace>/left_canister/cmd_pos` and `/<robot_namespace>/right_canister/cmd_pos`.
- For SLAM vs ground-truth pose comparison, use the same `world:=...` in both simulation and SLAM bringup, because `slam_bringup` sets `/robot/slam/pose.header.frame_id` from `world`.
- For vertical alignment, keep SLAM init pose at `x:=0.0 y:=0.0 z:=0.0` and use `slam_pose_offset_z:=0.35` (or your actual spawn height) so `/robot/slam/pose` is directly comparable with `/robot/ground_truth/pose`.
