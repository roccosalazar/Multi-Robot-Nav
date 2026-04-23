# CSLAM Project Summary

## Overview

CSLAM (Collaborative SLAM) is a **decentralized multi-robot SLAM system** built on ROS 2. It enables multiple robots to collaboratively build and optimize a shared map through inter-robot loop closure detection and distributed pose graph optimization. The system supports various sensor types including RGB-D cameras, stereo cameras, and LiDAR sensors.

**Repository:** [Swarm-SLAM](https://github.com/Swarm-SLAM) (Forked from [cslam](https://github.com/cslam))

## Architecture

The system follows a **front-end / back-end** architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS 2 Network                           │
│  (Zenoh / DDS for inter-robot communication)                │
└─────────────────────────────────────────────────────────────┘
         │                    │                    │
┌────────▼─────┐    ┌────────▼─────┐    ┌────────▼─────┐
│   Robot 0    │    │   Robot 1    │    │   Robot N    │
├──────────────┤    ├──────────────┤    ├──────────────┤
│ Front-end:   │    │ Front-end:   │    │ Front-end:   │
│ - Sensor     │    │ - Sensor     │    │ - Sensor     │
│   Handler    │    │ - Map Mgr    │    │ - Map Mgr    │
│ - Map Mgr    │    │              │    │              │
│              │    │              │    │              │
│ Back-end:    │◄──►│ Back-end:    │◄──►│ Back-end:    │
│ - Decentral- │    │ - Decentral- │    │ - Decentral- │
│   ized PGO   │    │   ized PGO   │    │   ized PGO   │
└──────────────┘    └──────────────┘    └──────────────┘
```

### Communication Layers

| Layer | Technology | Purpose |
|-------|------------|---------|
| **Intra-robot** | ROS 2 topics/services | Front-end ↔ Back-end within a robot |
| **Inter-robot** | Zenoh (default) or DDS | Pose graph exchange, descriptor sharing, neighbor discovery |
| **Transport** | Zenoh with multicast | Reliable multi-robot communication |

## Project Structure

```
Swarm-SLAM/
├── src/
│   ├── cslam/                          # Main package
│   │   ├── include/cslam/
│   │   │   ├── back_end/
│   │   │   │   ├── decentralized_pgo.h   # Pose graph optimizer
│   │   │   │   ├── gtsam_utils.h         # GTSAM ↔ ROS message conversion
│   │   │   │   └── utils/
│   │   │   │       ├── logger.h          # Logging/metrics
│   │   │   │       └── simulated_rendezvous.h  # Rendezvous simulation
│   │   │   └── front_end/
│   │   │       ├── map_manager.h         # Keyframe management
│   │   │       ├── rgbd_handler.h        # RGB-D sensor handler
│   │   │       ├── stereo_handler.h      # Stereo sensor handler
│   │   │       ├── sensor_handler_interface.h
│   │ │       └── utils/
│   │ │           └── depth_traits.h      # Depth image processing
│   │   ├── src/
│   │   │   ├── back_end/
│   │   │   │   ├── decentralized_pgo.cpp # PGO implementation
│   │   │   │   ├── gtsam_utils.cpp       # Conversion utilities
│   │   │   │   └── utils/
│   │   │   │       ├── logger.cpp
│   │   │   │       └── simulated_rendezvous.cpp
│   │   │   └── front_end/
│   │   │       ├── map_manager.cpp
│   │   │       ├── rgbd_handler.cpp
│   │   │       ├── stereo_handler.cpp
│   │   │       └── visualization_utils.cpp
│   │   ├── cslam/                       # Python nodes
│   │   │   ├── lidar_handler_node.py     # LiDAR front-end node
│   │   │   ├── loop_closure_detection_node.py  # Loop closure node
│   │   │   ├── loop_closure_sparse_matching.py # Sparse matching
│   │   │   ├── neighbors_manager.py      # Neighbor discovery
│   │   │   ├── broker.py                 # Communication broker
│   │   │   ├── lidar_pr/                 # LiDAR place recognition
│   │   │   │   ├── scancontext.py        # ScanContext descriptors
│   │   │   │   ├── scancontext_matching.py
│   │   │   │   └── icp_utils.py          # ICP registration
│   │   │   ├── vpr/                      # Visual place recognition
│   │   │   │   ├── cosplace.py           # Co-SPlace integration
│   │   │   │   └── netvlad.py            # NetVLAD descriptors
│   │   │   ├── mac/                      # Memory-aware communication
│   │   │   │   └── mac.py                # Bandwidth management
│   │   │   └── utils/
│   │   │       ├── misc.py               # Miscellaneous utilities
│   │   │       └── point_cloud2.py       # Point cloud utilities
│   │   └── models/                       # Pre-trained models
│   ├── cslam_interfaces/                # ROS 2 message definitions
│   │   ├── cslam_common_interfaces/
│   │   │   └── msg/                     # ~20 message types
│   │   └── cslam_zenoh_interfaces/      # Zenoh-specific messages
│   └── cslam_zenoh/                     # Zenoh transport layer
├── config/
│   ├── rendezvous/                      # Rendezvous simulation configs
│   ├── s3e/                             # S3E dataset configs
│   ├── kitti_stereo.yaml                # KITTI stereo config
│   ├── kitti_lidar.yaml                 # KITTI LiDAR config
│   ├── realsense_rgbd.yaml              # RealSense RGB-D config
│   └── ouster_lidar.yaml                # Ouster LiDAR config
├── launch/                              # ROS 2 launch files
├── experiments/                         # Experiment configurations
│   ├── datasets_experiments/
│   ├── robot_experiments/
│   ├── odometry/
│   └── sensors/
└── docker/                              # Docker configurations
```

## Core Components

### 1. Front-End: Sensor Handlers

The front-end processes raw sensor data, generates keyframes, and detects loop closures.

#### Sensor Handler Hierarchy

```
ISensorHandler (interface)
    │
    ├── RGBDHandler
    │   ├── rgbd_callback()           # Receives RGB + depth + odometry
    │   ├── generate_new_keyframe()   # Keyframe selection policy
    │   ├── send_keyframe()           # Publishes to Python node
    │   ├── compute_local_descriptors() # 3D feature extraction
    │   └── send_visualization()      # RViz visualization
    │
    └── StereoHandler (extends RGBDHandler)
        └── stereo_callback()         # Receives left + right images
```

#### Map Manager

```cpp
template <class DataHandlerType>
class MapManager : public IMapManager {
    // Manages keyframe queue, loop closure detection pipeline
    // Template parameter: StereoHandler or RGBDHandler
};
```

**Key responsibilities:**
- Receives keyframes from RTAB-Map
- Generates keypoints from frames
- Sends/receives keypoints from other robots
- Computes geometric verification for loop closures

### 2. Back-End: Decentralized Pose Graph Optimization

The back-end implements a **decentralized pose graph optimization** using GTSAM.

#### State Machine

```
┌─────────────────────────────┐
│           IDLE              │
└────────────┬────────────────┘
             │
             ▼
┌─────────────────────────────┐
│ WAITING_FOR_NEIGHBORS_INFO  │  ← Neighbor discovery via Zenoh
└────────────┬────────────────┘
             │
             ▼
┌─────────────────────────────┐
│    POSEGRAPH_COLLECTION     │  ← Request pose graphs from neighbors
└────────────┬────────────────┘
             │
             ▼
┌─────────────────────────────┐
│WAITING_FOR_NEIGHBORS_POSEGR│  ← Wait for responses
└────────────┬────────────────┘
             │
             ▼
┌─────────────────────────────┐
│   START_OPTIMIZATION        │  ← Check connectivity, select optimizer
└────────────┬────────────────┘
             │
             ▼
┌─────────────────────────────┐
│      OPTIMIZATION           │  ← GTSAM iSAM2 optimization
└────────────┬────────────────┘
             │
             └─→ Share results → IDLE
```

#### Optimizer Selection

The system uses a **priority-based optimizer selection**:
- Default: robot with the **lowest ID** becomes the optimizer
- The optimizer aggregates all pose graphs, runs optimization, and shares results

#### GTSAM Integration

```cpp
// Key GTSAM types used:
gtsam::NonlinearFactorGraph   // Pose graph factors
gtsam::Values                 // Pose estimates
gtsam::Pose3                  // 3D poses
gtsam::BetweenFactor          // Relative pose constraints
gtsam::GncOptimizer          # Generalized censing optimizer
```

**Key conversion utilities** (`gtsam_utils.h`):
- `odometry_msg_to_pose3()` — ROS odometry → GTSAM pose
- `transform_msg_to_pose3()` — ROS transform → GTSAM pose
- `gtsam_values_to_msg()` — GTSAM values → ROS messages
- `edges_msg_to_gtsam()` — ROS edges → GTSAM factors

### 3. Communication Layer

#### Zenoh Transport

The system uses **Zenoh** for inter-robot communication, providing:
- Multicast support for neighbor discovery
- Reliable message delivery
- Low-latency communication

#### Neighbor Management

```python
# neighbors_manager.py
class NeighborsManager:
    # Discovers neighbors via Zenoh
    # Maintains list of connected robots
    # Publishes current neighbors to back-end
```

#### Memory-Aware Communication (MAC)

```python
# mac/mac.py
# Manages communication bandwidth
# Prioritizes critical messages
# Implements sparsification strategies
```

### 4. Python Nodes

#### LiDAR Handler Node

```python
# lidar_handler_node.py
# Handles LiDAR data processing
# Generates ScanContext descriptors
# Performs loop closure detection
```

#### Loop Closure Detection

```python
# loop_closure_detection_node.py
# Receives descriptors from multiple robots
# Performs matching using various strategies
# Publishes inter-robot loop closures
```

#### Place Recognition

| Module | Algorithm | Purpose |
|--------|-----------|---------|
| `lidar_pr/scancontext.py` | ScanContext | LiDAR place recognition |
| `vpr/cosplace.py` | Co-SPlace | Visual place recognition |
| `vpr/netvlad.py` | NetVLAD | Global visual descriptors |

## ROS 2 Messages

### Common Interfaces (`cslam_common_interfaces`)

| Message | Purpose |
|---------|---------|
| `PoseGraph` | Complete pose graph with edges and values |
| `PoseGraphEdge` | Edge between two poses (MultiRobotKey) |
| `PoseGraphValue` | Pose estimate for a keyframe |
| `InterRobotLoopClosure` | Loop closure between two robots |
| `IntraRobotLoopClosure` | Loop closure within one robot |
| `GlobalDescriptor` | Global place recognition descriptor |
| `KeyframeOdom` | Keyframe ID + odometry data |
| `OptimizationResult` | Optimized pose estimates |
| `MultiRobotKey` | Robot ID + keyframe ID pair |
| `LocalDescriptorsRequest` | Request for local descriptors |
| `LocalImageDescriptors` | Local image descriptors for matching |
| `LocalPointCloudDescriptors` | Local point cloud descriptors |
| `OptimizerState` | Current state of the optimizer |

### Zenoh Interfaces (`cslam_zenoh_interfaces`)

Custom messages for Zenoh transport layer communication.

## Configuration

### Sensor Configuration

Each sensor type has a dedicated YAML configuration:

```yaml
# Example: kitti_stereo.yaml
robot_id: 0
max_nb_robots: 4
sensor_type: stereo
keyframe_period: 0.5
loop_closure_detection_period: 1.0
optimization_period: 5000  # ms
```

### Rendezvous Simulation

The system supports **simulated rendezvous scenarios** for testing:

```cpp
// simulated_rendezvous.h
class SimulatedRendezVous {
    // Reads schedule file
    // Determines when robots are "alive"
    // Simulates network connectivity
};
```

## Key Algorithms

### 1. Decentralized PGO

1. Each robot maintains its local pose graph
2. Periodically, robots exchange pose graphs with neighbors
3. An optimizer is selected (lowest ID by default)
4. The optimizer aggregates all graphs and runs GTSAM optimization
5. Results are shared with all participating robots

### 2. Loop Closure Detection

1. **Global descriptors** are computed for each keyframe
2. Descriptors are shared with neighboring robots
3. **Matching** is performed using similarity metrics
4. **Geometric verification** validates potential loop closures
5. Valid loop closures are added to the pose graph

### 3. ScanContext (LiDAR)

1. LiDAR scans are converted to **ScanContext** representations
2. **Ring and sector-based** feature extraction
3. **Fast matching** using contextual information
4. **ICP refinement** for precise alignment

## Building & Running

### Build

```bash
colcon build --packages-select cslam cslam_interfaces cslam_zenoh
source install/setup.bash
```

### Run (Example)

```bash
# Launch with Zenoh transport
ros2 launch cslam multi_robot_slam.launch.py \
    sensor_type:=stereo \
    config_file:=kitti_stereo.yaml \
    robot_id:=0
```

## Dependencies

### C++ Dependencies
- **GTSAM** — Graph SLAM optimization
- **RTAB-Map** — Real-time appearance-based mapping
- **OpenCV** — Computer vision utilities
- **Eigen** — Linear algebra

### Python Dependencies
- **NumPy** — Numerical computing
- **PyTorch** — Deep learning (for place recognition)
- **Open3D** — 3D data processing
- **Zenoh-Python** — Communication transport

### ROS 2 Dependencies
- `rclcpp` / `rclpy`
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`
- `tf2_ros`, `image_transport`
- `message_filters`

## Experiment Framework

The `experiments/` directory contains configurations for:
- **Dataset experiments** (KITTI, EuRoC, etc.)
- **Robot experiments** (physical robot deployments)
- **Odometry benchmarks**
- **Sensor comparisons**

## Logging & Metrics

The `Logger` class tracks:
- Pose graph statistics
- Optimization time
- Communication overhead
- Loop closure success/failure rates
- GPS-based error metrics (when available)

## Design Patterns

1. **Template Method** — `MapManager<DataHandlerType>` for different sensors
2. **State Machine** — Optimization pipeline states
3. **Observer** — ROS 2 callbacks for data processing
4. **Strategy** — Different place recognition algorithms
5. **Factory** — Sensor handler creation based on configuration

## Future Extensions

- Support for additional sensor types (event cameras, etc.)
- Improved optimizer selection strategies
- Adaptive communication based on network conditions
- Integration with navigation stacks
