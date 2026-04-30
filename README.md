# Multi-Robot-Nav

Multi-Robot-Nav is a ROS 2 simulation and evaluation repository for studying two collaborative LiDAR SLAM architectures on simulated Clearpath Warthog-class robots: Multi-Robot Graph SLAM and Swarm-SLAM.

This repository combines four main parts:

1. Clearpath simulation and robot-generation tooling:
   `clearpath_ws/`, `robots/`
2. Multi-Robot Graph SLAM:
   `Multi-Robot-Graph-SLAM/`, `mrg_slam`
3. Swarm-SLAM / CSLAM:
   `Swarm-SLAM/`, `cslam`, `cslam_common_interfaces`
4. Local project workspace:
   `workspace/` with bringup, evaluation, TF bridge, and visualization helpers

The current custom logic in this repository supports:

- spawning one or more simulated Warthog-class robots with reproducible poses;
- selecting between the MRG SLAM pipeline and the Swarm-SLAM / CSLAM pipeline after the robots are spawned;
- reusing the MRG SLAM scan-matching frontend inside the Swarm-SLAM / CSLAM workflow so both collaborative pipelines are compared on the same LiDAR odometry frontend;
- running both pipelines in single-robot and three-robot configurations;
- extracting comparable ground-truth and SLAM pose streams for evaluation;
- visualizing MRG SLAM and CSLAM outputs in RViz.

## What The Project Is Building

The default team is:

- `r0`
- `r1`
- `r2`

Each robot is a Clearpath Warthog-class `w200` platform with a 3D LiDAR payload. Gazebo provides simulation and ground truth, ROS 2 namespaces isolate per-robot topics and frames, shared/global TF is used where required by the SLAM stacks, and the evaluation helpers publish comparable pose streams for MRG SLAM and Swarm-SLAM / CSLAM.

## Repository Layout

- `robots/`: tracked robot setups for `r0`, `r1`, and `r2`, plus the shared canister payload Xacro.
- `clearpath_ws/`: vendored Clearpath stack used for robot generation and Gazebo simulation.
- `Multi-Robot-Graph-SLAM/`: vendored MRG SLAM stack and its dependencies.
- `Swarm-SLAM/`: vendored Swarm-SLAM / CSLAM stack.
- `workspace/src/musketeers_bringup/`: project launch wrappers, CSLAM config, and RViz profiles.
- `workspace/src/slam_evaluation/`: ground-truth extraction, SLAM pose publishing, CSLAM TF bridging, and RViz visualization helpers.
- `source_workspaces.sh`: helper that sources the local install spaces.

## Vendored Upstream Modifications

The vendored repositories are tracked directly inside this monorepo. The lists below summarize files modified in the monorepo relative to the vendored import history, not a formal upstream-to-upstream diff. They are derived from the monorepo Git history and can therefore include paths that were later renamed, removed, or only exist on another branch. For `clearpath_ws/` and `Multi-Robot-Graph-SLAM/`, the import baseline is the first root commit `1567e12d88ab755ff929db86dbbc4411d80b86ee`. For `Swarm-SLAM/`, the import baseline is `27cf4ede77d260bd5f021b0581aec27566ebcbbf`.

### Clearpath stack

Summary:

- TF and namespace normalization for generated robot bringup.
- Clearpath/Gazebo bridge integration for shared `/tf`.
- Generated robot config behavior and sensor launch adjustments.
- W200 color and payload-related customization support.
- Simulation world tuning and local performance report artifacts.

Modified paths under `clearpath_ws/`:

- `clearpath_ws/dependencies.repos` (removed from the tracked tree)
- `clearpath_ws/src/clearpath_common/clearpath_control/config/w200/control.yaml`
- `clearpath_ws/src/clearpath_common/clearpath_control/launch/control.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_control/launch/localization.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_control/launch/teleop_base.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_control/launch/teleop_joy.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_generator_common/clearpath_generator_common/common.py`
- `clearpath_ws/src/clearpath_common/clearpath_generator_common/clearpath_generator_common/description/platform.py`
- `clearpath_ws/src/clearpath_common/clearpath_generator_common/clearpath_generator_common/param/platform.py`
- `clearpath_ws/src/clearpath_common/clearpath_manipulators/launch/moveit.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_manipulators_description/launch/description.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_platform_description/launch/description.launch.py`
- `clearpath_ws/src/clearpath_common/clearpath_platform_description/urdf/generic/gazebo.urdf.xacro`
- `clearpath_ws/src/clearpath_common/clearpath_platform_description/urdf/w200/w200.urdf.xacro`
- `clearpath_ws/src/clearpath_common/clearpath_platform_description/urdf/w200/wheels/wheel.urdf.xacro`
- `clearpath_ws/src/clearpath_common/clearpath_sensors_description/urdf/velodyne_lidar.urdf.xacro`
- `clearpath_ws/src/clearpath_config/clearpath_config/platform/platform.py`
- `clearpath_ws/src/clearpath_config/setup.py`
- `clearpath_ws/src/clearpath_simulator/clearpath_generator_gz/clearpath_generator_gz/launch/generator.py`
- `clearpath_ws/src/clearpath_simulator/clearpath_generator_gz/clearpath_generator_gz/launch/sensors.py`
- `clearpath_ws/src/clearpath_simulator/clearpath_gz/worlds/pipeline.sdf`
- `clearpath_ws/src/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf`

### Multi-Robot-Graph-SLAM

Summary:

- Clearpath topic and frame integration.
- Configurable TF topic strategy and frame-prefix handling.
- Scan-matching-only launch support used by the CSLAM pipeline.
- Config updates for `r0/r1/r2` and Clearpath LiDAR topics.

Modified paths under `Multi-Robot-Graph-SLAM/`:

- `Multi-Robot-Graph-SLAM/src/mrg_slam/CMakeLists.txt`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/apps/prefiltering_component.cpp`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/config/mrg_slam.yaml`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/config/r0.yaml`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/config/scanmatching.yaml`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/config/swarm_slam.yaml`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/config/swarm_team.yaml`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/launch/mrg_slam.launch.py`
- `Multi-Robot-Graph-SLAM/src/mrg_slam/launch/mrg_slam_v2.launch.py`

### Swarm-SLAM

Summary:

- Local CSLAM patches for debugging, QoS, and loop-detection behavior.
- Backend changes around optimizer state, anchor handling, and visualization support.
- Integration with scan-matching odometry through the local bringup layer.

Modified paths under `Swarm-SLAM/`:

- `Swarm-SLAM/docs/architecture/PROJECT_SUMMARY.md`
- `Swarm-SLAM/docs/architecture/architecture_overview.md`
- `Swarm-SLAM/docs/architecture/backend_architecture.md`
- `Swarm-SLAM/src/cslam/cslam/global_descriptor_loop_closure_detection.py`
- `Swarm-SLAM/src/cslam/cslam/lidar_handler_node.py`
- `Swarm-SLAM/src/cslam/cslam/loop_closure_detection_node.py`
- `Swarm-SLAM/src/cslam/include/cslam/back_end/decentralized_pgo.h`
- `Swarm-SLAM/src/cslam/src/back_end/decentralized_pgo.cpp`

## `workspace/src` packages

### `musketeers_bringup`

Purpose: launch orchestration for simulation, robot spawning, and the two SLAM pipelines.

Important launch files:

- `spawn_world.launch.py`
- `spawn_robot.launch.py`
- `spawn_team.launch.py`
- `mrg_single_slam.launch.py`
- `mrg_multi_slam.launch.py`
- `swarm_single_slam.launch.py`
- `swarm_multi_slam.launch.py`

Important config:

- `config/cslam_lidar.yaml`

The CSLAM wrappers also default `lidar_odometry_config:=scanmatching.yaml`, which is resolved from the vendored `mrg_slam` package.
In practice, the Swarm-SLAM frontend in this repository is fed by `mrg_slam_v2.launch.py`, so CSLAM consumes `scan_matching_odometry/odom` and `scan_matching_odometry/aligned_points` from the same scan-matching frontend used for the comparative setup.

Important RViz profiles:

- `rviz/r0.rviz`
- `rviz/multi_slam.rviz`

### `slam_evaluation`

Purpose: expose comparable pose outputs and RViz helpers for evaluation.

Useful nodes:

- `ground_truth_pose_extractor.py`: filters Gazebo dynamic pose output into `PoseStamped` ground truth.
- `slam_pose_publisher.py`: publishes `PoseStamped` SLAM poses from TF.
- `cslam_odom_tf_bridge.py`: anchors CSLAM optimized poses to local odometry through `map->odom`.
- `cslam_pose_graph_rviz.py`: converts CSLAM pose graphs into RViz markers.
- `cslam_keyframe_cloud_viewer.py`: visualizes CSLAM keyframe clouds in RViz.

## Build and environment setup

Build order used in this repository:

```bash
cd clearpath_ws
colcon build --symlink-install

cd ../Multi-Robot-Graph-SLAM
export MAKEFLAGS="-j 2"
colcon build --symlink-install --parallel-workers 2 --executor sequential

cd ../Swarm-SLAM
export MAKEFLAGS="-j 2"
colcon build --symlink-install --parallel-workers 2 --executor sequential

cd ../workspace
colcon build --symlink-install

cd ..
```

Source ROS 2 and the local workspaces:

```bash
source /opt/ros/humble/setup.bash
source source_workspaces.sh
```

`source_workspaces.sh` currently sources:

```bash
source ./workspace/install/setup.bash
source ./clearpath_ws/install/setup.bash
source ./Multi-Robot-Graph-SLAM/install/setup.bash
source ./Swarm-SLAM/install/setup.bash
```

## Typical Launch Workflow

### 1. Start a Gazebo world

```bash
ros2 launch musketeers_bringup spawn_world.launch.py world:=warehouse
```

`world:=...` selects the simulation world.

### 2. Spawn one robot

```bash
ros2 launch musketeers_bringup spawn_robot.launch.py \
  robot_name:=r0 \
  x:=0.0 y:=0.0 z:=0.35 yaw:=0.0 \
  world:=warehouse \
  ground_truth_pose:=true
```

`robot_name` selects the setup under `robots/<robot_name>`. `ground_truth_pose:=true` enables Gazebo ground-truth pose extraction for that robot.

### 3. Spawn the full robot team

```bash
ros2 launch musketeers_bringup spawn_team.launch.py world:=warehouse
```

This launches the tracked `r0`, `r1`, and `r2` setups with fixed delays and reproducible poses.

### 4. Run MRG SLAM for one robot

```bash
ros2 launch musketeers_bringup mrg_single_slam.launch.py \
  robot_name:=r0 \
  config:=r0.yaml \
  use_sim_time:=true \
  world:=warehouse \
  x:=0.0 y:=0.0 z:=0.0 \
  slam_pose_offset_x:=0.0 slam_pose_offset_y:=0.0 slam_pose_offset_z:=0.35
```

### 5. Run MRG SLAM for the full team

```bash
ros2 launch musketeers_bringup mrg_multi_slam.launch.py \
  use_sim_time:=true \
  world:=warehouse
```

`mrg_multi_slam.launch.py` defaults to `config:=swarm_team.yaml`.

### 6. Run Swarm-SLAM / CSLAM for one robot

```bash
ros2 launch musketeers_bringup swarm_single_slam.launch.py \
  robot_name:=r0 \
  robot_id:=0 \
  max_nb_robots:=1
```

### 7. Run Swarm-SLAM / CSLAM for the full team

```bash
ros2 launch musketeers_bringup swarm_multi_slam.launch.py \
  start_graph_viewer:=true \
  start_cloud_viewer:=true
```

This starts the CSLAM pipeline for `r0`, `r1`, and `r2`. In multi-robot runs, enable the pose-graph and keyframe-cloud viewers only once.

### 8. Open RViz

```bash
rviz2 -d $HOME/Multi-Robot-Nav/workspace/src/musketeers_bringup/rviz/r0.rviz
```

```bash
rviz2 -d $HOME/Multi-Robot-Nav/workspace/src/musketeers_bringup/rviz/multi_slam.rviz
```

## Practical Notes

- `spawn_robot.launch.py` defaults `generate:=false`, so Clearpath-generated artifacts are expected to already exist unless `generate:=true` is requested explicitly.
- `spawn_team.launch.py` launches the tracked `r0`, `r1`, and `r2` setups with fixed 7-second staging delays after the first robot.
- Robot setup files and generated Clearpath artifacts may contain absolute `/home/ubuntu/Multi-Robot-Nav/...` paths.
- Workspace launch defaults are aligned to `world:=warehouse`.
- W200 body color is configured from each robot setup through `platform.color`.
- The canister command topics are `/<robot_namespace>/left_canister/cmd_pos` and `/<robot_namespace>/right_canister/cmd_pos`.
- For SLAM-versus-ground-truth comparison, keep the same `world:=...` in the simulation and SLAM launches.
- For MRG SLAM vertical alignment, keep the SLAM init pose at `x:=0.0 y:=0.0 z:=0.0` and use `slam_pose_offset_z:=0.35` or the actual spawn height.
- For CSLAM runs, keep `cslam_lidar.yaml` aligned with the actual scan-matching outputs from `mrg_slam/config/scanmatching.yaml`; this repository intentionally uses the MRG SLAM scan-matching frontend for Swarm-SLAM so the comparison stays coherent.
- In multi-robot CSLAM runs, enable `start_graph_viewer:=true` and `start_cloud_viewer:=true` only once.
