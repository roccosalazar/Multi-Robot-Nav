# Swarm-SLAM Architecture Overview

## Project Summary

Swarm-SLAM is a **multi-robot collaborative SLAM system** that enables multiple robots to build a jointly optimized map using decentralized pose graph optimization. The system supports multiple sensor types (RGB-D cameras, stereo cameras, and LiDAR) and uses RTAB-Map for front-end feature extraction and GTSAM for back-end optimization.

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Swarm-SLAM System                                │
│                                                                         │
│  ┌─────────────────────┐    ┌─────────────────────────────────────────┐ │
│  │    FRONT-END        │    │            BACK-END                     │ │
│  │  (Per Robot)        │    │         (Per Robot)                     │ │
│  │                     │    │                                         │ │
│  │  Sensor Handler     │    │  DecentralizedPGO                       │ │
│  │  ├─ RGBDHandler     │    │  ├─ Pose Graph Management              │ │
│  │  ├─ StereoHandler   │    │  ├─ GTSAM Optimization                 │ │
│  │  └─ LidarHandler    │    │  ├─ Neighbor Discovery                 │ │
│  │                     │    │  └─ Coordinate Alignment               │ │
│  │  Responsibilities:  │    │                                         │ │
│  │  • Sensor data sync │    │  Responsibilities:                      │ │
│  │  • Keyframe gen.    │    │  • Local pose graph                     │ │
│  │  • Feature extract. │    │  • Inter-robot loop closures            │ │
│  │  • Local descriptors│    │  • Distributed optimization             │ │
│  │  • Loop detection   │    │  • TF broadcasting                      │ │
│  └─────────────────────┘    └─────────────────────────────────────────┘ │
│                                                                         │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                    COMMUNICATION LAYER                            │  │
│  │  • ROS 2 Topics (keyframes, odometry, descriptors)               │  │
│  │  • Inter-robot loop closure messages                              │  │
│  │  • Pose graph exchange                                           │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Core Components

### 1. Front-End: Sensor Handlers

The front-end processes raw sensor data, generates keyframes, extracts features, and detects loop closures.

#### Interface
- **`ISensorHandler`** (`include/cslam/front_end/sensor_handler_interface.h`)
  - Pure virtual base class
  - Defines `process_new_sensor_data()` callback

#### Implementations

| Handler | Language | Sensors | Key Features |
|---------|----------|---------|--------------|
| **RGBDHandler** | C++ | RGB + Depth + Odometry | Visual features, depth masking, PnP-based keyframe selection |
| **StereoHandler** | C++ | Left/Right Stereo + Odometry | Disparity-based depth, visual features |
| **LidarHandler** | Python | PointCloud2 + Odometry | ICP-based registration, voxel grid filtering |

#### RGBDHandler Details
```
File: src/front_end/rgbd_handler.cpp
```
- **Data Synchronization**: Uses `message_filters::Synchronizer` with approximate time sync for RGB, depth, camera info, and odometry
- **Keyframe Generation**: PnP-based inlier ratio threshold (`keyframe_generation_ratio_threshold`)
- **Feature Extraction**: RTAB-Map's `Feature2D` (ORB/SIFT/AKAZE depending on config)
- **Local Descriptors**: 3D keypoints with descriptors stored in `local_descriptors_map_`
- **Visualization**: Optional point cloud publishing with voxel subsampling

#### LidarHandler Details
```
File: cslam/lidar_handler_node.py
```
- **Data Synchronization**: `ApproximateTimeSynchronizer` for point cloud + odometry
- **Keyframe Generation**: Distance-based threshold (`keyframe_generation_ratio_distance`)
- **Registration**: ICP-based (`cslam/lidar_pr/icp_utils.py`)
- **Filtering**: Voxel grid downsampling, covariance-based odometry rejection

### 2. Back-End: Decentralized Pose Graph Optimization

#### DecentralizedPGO
```
File: include/cslam/back_end/decentralized_pgo.h
```

**State Machine**:
```
IDLE
  ↓
WAITING_FOR_NEIGHBORS_INFO
  ↓
POSEGRAPH_COLLECTION
  ↓
WAITING_FOR_NEIGHBORS_POSEGRAPHS
  ↓
START_OPTIMIZATION
  ↓
OPTIMIZATION
  ↓ (loop back)
IDLE
```

**Key Responsibilities**:
1. **Local Pose Graph**: Maintains intra-robot constraints from odometry and loop closures
2. **Neighbor Management**: Discovers nearby robots via heartbeat messages
3. **Pose Graph Exchange**: Requests/receives pose graphs from neighbors
4. **Coordinate Alignment**: Aligns different robot coordinate frames using inter-robot loop closures
5. **Optimization**: Uses GTSAM with GNC (Gain Scheduling) optimizer
6. **TF Broadcasting**: Publishes optimized transforms

**Optimization Pipeline**:
```cpp
// Key methods in DecentralizedPGO
void start_optimization()           // Trigger optimization
void build_pose_graph()             // Construct GTSAM factor graph
void align_pose_graphs()            // Align neighbor graphs to local frame
void optimize()                    // Run GTSAM GNC optimizer
void broadcast_results()           // Publish optimized poses
```

### 3. Communication Messages

All custom messages are defined in `cslam_common_interfaces`:

| Message | Purpose |
|---------|---------|
| `KeyframeRGB` / `KeyframePointCloud` | Keyframe data for loop detection |
| `KeyframeOdom` | Odometry with keyframe ID |
| `LocalImageDescriptors` / `LocalPointCloudDescriptors` | Local feature descriptors |
| `LocalDescriptorsRequest` | Request descriptors from another robot |
| `LocalKeyframeMatch` | Local match result between robots |
| `InterRobotLoopClosure` | Loop closure between different robots |
| `IntraRobotLoopClosure` | Loop closure within same robot |
| `PoseGraph` | Full pose graph for exchange |
| `OptimizationResult` | Optimized pose estimates |
| `RobotIdsAndOrigin` | Neighbor discovery |
| `VizPointCloud` | Visualization point clouds |

---

## Data Flow

### Per-Robot Pipeline

```
┌──────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Raw Sensors  │────▶│  Sensor Handler   │────▶│  Keyframe Gen.  │
│  (ROS 2)     │     │  (Sync + Queue)   │     │  (Threshold)    │
└──────────────┘     └──────────────────┘     └────────┬────────┘
                                                       │
┌──────────────┐     ┌──────────────────┐     ┌────────▼────────┐
│  TF Broadcast │◀────│  DecentralizedPGO │◀────│  Feature Extr.  │
│  (Optimized)  │     │  (Back-End)      │     │  + Loop Detect. │
└──────────────┘     └──────────────────┘     └─────────────────┘
```

### Multi-Robot Collaboration

```
Robot A                          Robot B
┌─────────────┐                 ┌─────────────┐
│ Front-End   │                 │ Front-End   │
│ • Publish   │                 │ • Publish   │
│   local     │                 │   local     │
│   descriptors│                │   descriptors│
└──────┬──────┘                 └──────┬──────┘
       │                              │
       │  LocalDescriptors            │
       └──────────────┬───────────────┘
                      │
       ┌──────────────▼───────────────┐
       │     Descriptor Matching      │
       │     (Cross-robot)            │
       └──────┬──────────────┬────────┘
              │              │
    InterRobotLoopClosure    InterRobotLoopClosure
              │              │
┌──────▼──────┐                 ┌──────▼──────┐
│ Back-End    │                 │ Back-End    │
│ • Add       │                 │ • Add       │
│   constraints│                │   constraints│
│ • Optimize  │◀────align──────▶│ • Optimize  │
└─────────────┘                 └─────────────┘
```

---

## Key Parameters

### Front-End
```yaml
frontend:
  max_queue_size: 100
  keyframe_generation_ratio_threshold: 0.5      # RGBD/Stereo
  keyframe_generation_ratio_distance: 0.5       # LiDAR
  pnp_min_inliers: 20
  color_image_topic: "color/image"
  depth_image_topic: "depth/image"
  odom_topic: "odom"
```

### Back-End
```yaml
backend:
  pose_graph_optimization_start_period_ms: 1000
  pose_graph_optimization_loop_period_ms: 100
  max_waiting_time_sec: 100
  enable_broadcast_tf_frames: true

neighbor_management:
  heartbeat_period_sec: 1.0

max_nb_robots: 4
robot_id: 0
```

### Visualization
```yaml
visualization:
  enable: false
  publishing_period_ms: 100
  voxel_size: 0.05
  max_range: 10.0
```

---

## File Structure

```
src/cslam/
├── include/cslam/
│   ├── back_end/
│   │   ├── decentralized_pgo.h        # Back-end optimizer
│   │   ├── gtsam_utils.h             # GTSAM utilities
│   │   └── utils/
│   │       ├── logger.h              # Logging utilities
│   │       └── simulated_rendezvous.h # Simulated robot meetings
│   └── front_end/
│       ├── sensor_handler_interface.h # Base interface
│       ├── rgbd_handler.h            # RGB-D handler
│       ├── stereo_handler.h          # Stereo handler
│       └── utils/
│           └── visualization_utils.h  # Viz helpers
├── src/
│   ├── back_end/
│   │   ├── decentralized_pgo.cpp     # Optimizer implementation
│   │   └── pose_graph_manager_node.cpp # Node entry point
│   └── front_end/
│       ├── rgbd_handler.cpp          # RGB-D implementation
│       ├── stereo_handler.cpp        # Stereo implementation
│       └── map_manager_node.cpp      # Node entry point
├── cslam/
│   ├── lidar_handler_node.py         # LiDAR handler (Python)
│   └── lidar_pr/
│       └── icp_utils.py             # ICP registration utilities
└── config/
    └── params.yaml                  # Default parameters
```

---

## Dependencies

| Dependency | Purpose |
|------------|---------|
| **RTAB-Map** | Feature extraction, visual odometry, loop detection |
| **GTSAM** | Pose graph optimization (GNC optimizer) |
| **ROS 2** | Communication, TF, node management |
| **OpenCV** | Image processing, feature detection |
| **PCL** | Point cloud processing |
| **cv_bridge** | ROS/OpenCV image conversion |
| **tf2_ros** | Coordinate transform management |

---

## Design Patterns

1. **Strategy Pattern**: `ISensorHandler` interface with multiple implementations (RGBD, Stereo, LiDAR)
2. **State Machine**: `DecentralizedPGO` uses explicit state transitions for optimization workflow
3. **Observer Pattern**: ROS 2 callbacks for asynchronous data processing
4. **Filter Chain**: `message_filters` for synchronized multi-sensor data
5. **Producer-Consumer**: Queue-based data processing in sensor handlers

---

## Optimization Workflow

1. **Local Map Building**: Each robot builds its own pose graph from odometry + intra-robot loop closures
2. **Neighbor Discovery**: Robots broadcast heartbeats to discover nearby robots
3. **Descriptor Exchange**: Robots share local descriptors for cross-robot feature matching
4. **Inter-Robot Loop Closures**: When matches are found, loop closure constraints are created
5. **Pose Graph Exchange**: Robots request and receive pose graphs from neighbors
6. **Coordinate Alignment**: Neighbor pose graphs are aligned to the local coordinate frame
7. **Joint Optimization**: GTSAM optimizes the combined pose graph with all constraints
8. **Result Distribution**: Optimized poses are broadcast for TF publishing

---

## Notes

- The system is **decentralized** — each robot runs its own optimizer
- **No central server** required for map fusion
- Supports **heterogeneous sensors** across robots
- **Python LiDAR handler** provides flexibility for rapid prototyping
- **Evaluation mode** enables GPS recording and logging for benchmarking