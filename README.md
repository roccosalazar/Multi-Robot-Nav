# Multi-Robot-Nav

This repository is a ROS 2 simulation project for testing multi-robot SLAM architectures in underground environments, using worlds inspired by the DARPA Subterranean (SubT) Challenge.

The testbed combines:
- Clearpath robot configurations (Warthog-based platforms)
- Multi-Robot Graph SLAM components
- A custom bringup workspace to spawn one or multiple robots in simulation

## Project Goal

The main objective is to evaluate and compare multi-robot SLAM pipelines in complex underground-like maps (tunnels, warehouses, and SubT-style scenarios), with reproducible robot spawn and launch workflows.

## Repository Layout

- `clearpath_ws/`: Clearpath ROS 2 stack and robot simulation dependencies
- `Multi-Robot-Graph-SLAM/`: Multi-robot graph SLAM packages
- `workspace/`: custom project packages, including `musketeers_bringup`
- `clearpath/`: robot setup folders (`aramis`, `athos`, `porthos`)
- `source_workspaces.sh`: helper script that sources all built workspaces

## Prerequisites

- Ubuntu Linux
- ROS 2 (recommended: Humble)
- Colcon
- Gazebo / Clearpath simulation dependencies required by the included packages

## Build Instructions

Build all three workspaces in this order:

```bash
cd clearpath_ws
colcon build --symlink-install

cd ../Multi-Robot-Graph-SLAM
colcon build --symlink-install

cd ../workspace
colcon build --symlink-install

cd ..
```

## Source Environment

After building, source ROS 2 and all local workspaces:

```bash
source /opt/ros/humble/setup.bash
source source_workspaces.sh
```

The `source_workspaces.sh` script loads:
- `workspace/install/setup.bash`
- `clearpath_ws/install/setup.bash`
- `Multi-Robot-Graph-SLAM/install/setup.bash`

## Launching Simulation

### 1) Spawn World

Launch the simulation world:

```bash
ros2 launch musketeers_bringup spawn_world.launch.py world:=<world_name>
```

Example:

```bash
ros2 launch musketeers_bringup spawn_world.launch.py world:=warehouse
```

### 2) Spawn a Single Robot

Launch one robot with custom pose:

```bash
ros2 launch musketeers_bringup spawn_robot.launch.py \
	robot_name:=<robot_name> \
	x:=<x> y:=<y> z:=<z> yaw:=<yaw> \
	world:=<world_name>
```

Common robot names:
- `aramis`
- `athos`
- `porthos`

Example:

```bash
ros2 launch musketeers_bringup spawn_robot.launch.py \
	robot_name:=aramis x:=0.0 y:=0.0 z:=0.3 yaw:=0.0 world:=warehouse
```

Available optional arguments in this launch file also include:
- `rviz:=true|false`
- `use_sim_time:=true|false`
- `generate:=true|false`

### 3) Spawn the Three Musketeers

Launch all three Clearpath Warthog robots with predefined poses:

```bash
ros2 launch musketeers_bringup spawn_musketeers.launch.py world:=<world_name>
```

Example:

```bash
ros2 launch musketeers_bringup spawn_musketeers.launch.py world:=warehouse
```

This launch starts:
- `aramis`
- `athos`
- `porthos`

with staggered spawn timing and relay nodes for TF and Velodyne topics.

## Notes

- Robot setup paths are expected under `~/clearpath/<robot_name>`.
- If a launch command cannot find packages, rebuild and re-source all workspaces.
- Keep terminals sourced with `source source_workspaces.sh` before running ROS 2 launch commands.
