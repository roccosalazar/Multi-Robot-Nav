# CSLAM / Swarm-SLAM Architecture Documentation

## 1. Overview

CSLAM / Swarm-SLAM is a **decentralized multi-robot collaborative SLAM system** built on **ROS 2**. It enables multiple robots to collaboratively build, align, and optimize a shared map through **inter-robot loop closure detection**, **pose graph exchange**, and **decentralized pose graph optimization**.

The system follows a **front-end / back-end architecture**:

* The **front-end** processes raw sensor data, generates keyframes, extracts features and descriptors, and detects intra-robot and inter-robot loop closures.
* The **back-end** maintains a pose graph, exchanges pose graph information with neighboring robots, performs collaborative pose graph optimization, and publishes optimized poses and TF frames.
* The **communication layer** supports intra-robot ROS 2 communication and inter-robot communication through **Zenoh by default** or **DDS**, with support for ad-hoc multi-robot networking.

Swarm-SLAM supports heterogeneous sensors:

* RGB-D cameras
* Stereo cameras
* LiDAR sensors

The system uses:

* **RTAB-Map** for front-end feature extraction, visual odometry support, loop detection, and sensor processing utilities
* **GTSAM** for back-end pose graph optimization
* **ROS 2** for node orchestration, topics, TF, timers, and messaging
* **Zenoh** for inter-robot communication in the Swarm-SLAM transport layer
* **Python front-end nodes** for LiDAR processing, loop closure detection, communication brokerage, and place recognition modules

Repository information from the source documents:

```markdown
Repository: [Swarm-SLAM](https://github.com/Swarm-SLAM)
Forked from: [cslam](https://github.com/cslam)
```

The Swarm-SLAM paper also describes the system as a ROS 2 open-source C-SLAM framework designed to be **scalable**, **flexible**, **decentralized**, and **sparse**, supporting LiDAR, stereo, and RGB-D sensing with sparse inter-robot loop closure prioritization. 

---

## 2. System Goals

The main goals of CSLAM / Swarm-SLAM are:

1. Build and maintain a **local pose graph** for each robot.
2. Exchange pose graph information with **neighboring robots** through peer-to-peer communication.
3. Detect **intra-robot loop closures** within one robot trajectory.
4. Detect **inter-robot loop closures** between trajectories of different robots.
5. Aggregate local and neighboring pose graphs during rendezvous events.
6. Execute **collaborative pose graph optimization** over the connected multi-robot graph.
7. Share optimized pose estimates with all participating robots.
8. Broadcast TF frames for visualization and integration with the rest of the robotic system.
9. Support heterogeneous sensors and arbitrary odometry sources.
10. Avoid reliance on a permanent central server or base station.
11. Support sparse and bandwidth-aware communication.
12. Support experiments, benchmarking, logging, GPS recording, and simulated rendezvous.

### 2.1 Swarm-Compatible Properties

The architecture is designed around four core properties:

| Property          | Meaning                                                                                                                                                                                                        |
| ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Scalable**      | The number of robots is not fixed in advance. Robots do not need permanent connectivity throughout the whole mission. Communication and computation budgets can be adapted to the available onboard resources. |
| **Flexible**      | The framework supports stereo cameras, RGB-D cameras, and LiDAR sensors. It is decoupled from the odometry source.                                                                                             |
| **Decentralized** | All computation is performed onboard robots. There is no permanent central authority. Robots communicate peer-to-peer.                                                                                         |
| **Sparse**        | Communication is reduced by prioritizing inter-robot loop closure candidates and avoiding redundant transfer of local features or point clouds.                                                                |

### 2.2 Back-End Goals

The `DecentralizedPGO` back-end node specifically aims to:

* Construct and maintain the robot-local pose graph.
* Receive odometry, intra-robot loop closures, and inter-robot loop closures.
* Discover and manage neighboring robots.
* Request and receive pose graphs from neighbors.
* Aggregate connected pose graphs.
* Add a prior / anchor for optimization.
* Run robust GTSAM optimization.
* Publish optimized estimates to participating robots.
* Broadcast TF transforms:

  * `map_{origin} → map_{local}`
  * `map_{origin} → latest_optimized_{local}`
  * `latest_optimized_{local} → current_{local}`

---

## 3. High-Level Architecture

### 3.1 Swarm-SLAM System Diagram

```text
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

### 3.2 ROS 2 Network / Multi-Robot Diagram

```text
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

### 3.3 Main Architectural Modules

| Module                         | Main Responsibility                                                                                                              |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------- |
| **Sensor handlers**            | Synchronize sensor data, generate keyframes, extract local descriptors, publish keyframe data.                                   |
| **Map manager**                | Manage keyframe queues, receive keyframes from RTAB-Map, generate keypoints, exchange keypoints, perform geometric verification. |
| **Loop closure detection**     | Match global descriptors, request local descriptors, verify loop closure candidates.                                             |
| **Neighbor management**        | Track robots in communication range, publish heartbeat messages, maintain exchanged-data bookkeeping.                            |
| **Communication broker**       | Coordinate inter-robot descriptor and vertex exchange during rendezvous.                                                         |
| **Memory-aware communication** | Manage bandwidth, prioritize critical messages, apply sparsification strategies.                                                 |
| **DecentralizedPGO**           | Maintain pose graphs, exchange graphs, aggregate connected graphs, optimize, publish results, broadcast TF.                      |
| **Visualization / logging**    | Publish RViz data, record metrics, export pose graphs, support benchmarking.                                                     |

### 3.4 Communication Layers

| Layer           | Technology              | Purpose                                                           |
| --------------- | ----------------------- | ----------------------------------------------------------------- |
| **Intra-robot** | ROS 2 topics / services | Front-end ↔ back-end communication within one robot.              |
| **Inter-robot** | Zenoh by default or DDS | Pose graph exchange, descriptor sharing, neighbor discovery.      |
| **Transport**   | Zenoh with multicast    | Reliable multi-robot communication and ad-hoc networking support. |

### 3.5 Per-Robot Data Flow

```text
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

---

## 4. Repository Structure

The source documents describe the repository using both package-level and workspace-level views. The merged structure is:

```text
Swarm-SLAM/
├── src/
│   ├── cslam/                          # Main package
│   │   ├── include/cslam/
│   │   │   ├── back_end/
│   │   │   │   ├── decentralized_pgo.h          # Back-end optimizer / DecentralizedPGO declaration
│   │   │   │   ├── gtsam_utils.h                # GTSAM ↔ ROS message conversion utilities
│   │   │   │   └── utils/
│   │   │   │       ├── logger.h                 # Logging / metrics utilities
│   │   │   │       └── simulated_rendezvous.h   # Simulated robot meetings / rendezvous
│   │   │   └── front_end/
│   │   │       ├── map_manager.h                # Keyframe management
│   │   │       ├── rgbd_handler.h               # RGB-D sensor handler
│   │   │       ├── stereo_handler.h             # Stereo sensor handler
│   │   │       ├── sensor_handler_interface.h   # Base sensor handler interface
│   │   │       └── utils/
│   │   │           ├── depth_traits.h           # Depth image processing
│   │   │           └── visualization_utils.h    # Visualization helpers
│   │   ├── src/
│   │   │   ├── back_end/
│   │   │   │   ├── decentralized_pgo.cpp        # Optimizer implementation
│   │   │   │   ├── gtsam_utils.cpp              # Conversion utility implementation
│   │   │   │   ├── pose_graph_manager_node.cpp  # Back-end node entry point
│   │   │   │   └── utils/
│   │   │   │       ├── logger.cpp
│   │   │   │       └── simulated_rendezvous.cpp
│   │   │   └── front_end/
│   │   │       ├── map_manager.cpp
│   │   │       ├── rgbd_handler.cpp             # RGB-D implementation
│   │   │       ├── stereo_handler.cpp           # Stereo implementation
│   │   │       ├── map_manager_node.cpp         # Front-end node entry point
│   │   │       └── visualization_utils.cpp
│   │   ├── cslam/                               # Python nodes
│   │   │   ├── lidar_handler_node.py            # LiDAR front-end node
│   │   │   ├── loop_closure_detection_node.py   # Loop closure node
│   │   │   ├── loop_closure_sparse_matching.py  # Sparse matching
│   │   │   ├── neighbors_manager.py             # Neighbor discovery
│   │   │   ├── broker.py                        # Communication broker
│   │   │   ├── lidar_pr/                        # LiDAR place recognition
│   │   │   │   ├── scancontext.py               # ScanContext descriptors
│   │   │   │   ├── scancontext_matching.py
│   │   │   │   └── icp_utils.py                 # ICP registration utilities
│   │   │   ├── vpr/                             # Visual place recognition
│   │   │   │   ├── cosplace.py                  # Co-SPlace integration
│   │   │   │   └── netvlad.py                   # NetVLAD descriptors
│   │   │   ├── mac/                             # Memory-aware communication
│   │   │   │   └── mac.py                       # Bandwidth management
│   │   │   └── utils/
│   │   │       ├── misc.py                      # Miscellaneous utilities
│   │   │       └── point_cloud2.py              # Point cloud utilities
│   │   └── models/                              # Pre-trained models
│   ├── cslam_interfaces/                        # ROS 2 message definitions
│   │   ├── cslam_common_interfaces/
│   │   │   └── msg/                             # ~20 common message types
│   │   └── cslam_zenoh_interfaces/              # Zenoh-specific messages
│   └── cslam_zenoh/                             # Zenoh transport layer
├── config/
│   ├── params.yaml                              # Default parameters
│   ├── cslam/
│   │   └── example.yaml                         # Example CSLAM configuration
│   ├── rendezvous/                              # Rendezvous simulation configs
│   ├── s3e/                                     # S3E dataset configs
│   ├── kitti_stereo.yaml                        # KITTI stereo config
│   ├── kitti_lidar.yaml                         # KITTI LiDAR config
│   ├── realsense_rgbd.yaml                      # RealSense RGB-D config
│   └── ouster_lidar.yaml                        # Ouster LiDAR config
├── launch/                                      # ROS 2 launch files
├── experiments/                                 # Experiment configurations
│   ├── datasets_experiments/
│   ├── robot_experiments/
│   ├── odometry/
│   └── sensors/
└── docker/                                      # Docker configurations
```

---

## 5. Core Components

### 5.1 Component Summary

| Component                     | Language | Package / Path                                                                     | Responsibility                                                                                         |
| ----------------------------- | -------- | ---------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `ISensorHandler`              | C++      | `include/cslam/front_end/sensor_handler_interface.h`                               | Pure virtual front-end interface defining `process_new_sensor_data()`.                                 |
| `RGBDHandler`                 | C++      | `include/cslam/front_end/rgbd_handler.h`, `src/front_end/rgbd_handler.cpp`         | RGB-D data synchronization, keyframe generation, feature extraction, local descriptors, visualization. |
| `StereoHandler`               | C++      | `include/cslam/front_end/stereo_handler.h`, `src/front_end/stereo_handler.cpp`     | Stereo image processing, disparity/depth handling, visual descriptors.                                 |
| `LidarHandler`                | Python   | `cslam/lidar_handler_node.py`                                                      | Point cloud processing, LiDAR keyframes, ScanContext descriptors, ICP registration.                    |
| `MapManager<DataHandlerType>` | C++      | `include/cslam/front_end/map_manager.h`, `src/front_end/map_manager.cpp`           | Keyframe queue management, loop closure pipeline, keypoint exchange, geometric verification.           |
| `DecentralizedPGO`            | C++      | `include/cslam/back_end/decentralized_pgo.h`, `src/back_end/decentralized_pgo.cpp` | Back-end pose graph management, graph exchange, optimization, TF broadcasting.                         |
| GTSAM utilities               | C++      | `include/cslam/back_end/gtsam_utils.h`, `src/back_end/gtsam_utils.cpp`             | Convert between ROS messages and GTSAM types.                                                          |
| `Logger`                      | C++      | `include/cslam/back_end/utils/logger.h`                                            | Metrics, pose graph logging, optimization timing, GPS-based error metrics.                             |
| `SimulatedRendezVous`         | C++      | `include/cslam/back_end/utils/simulated_rendezvous.h`                              | Simulated communication/rendezvous scheduling.                                                         |
| `NeighborsManager`            | Python   | `cslam/neighbors_manager.py`                                                       | Neighbor discovery through Zenoh, connected robot list, current-neighbor publication.                  |
| Broker                        | Python   | `cslam/broker.py`                                                                  | Temporary communication broker for descriptor / vertex exchange.                                       |
| MAC                           | Python   | `cslam/mac/mac.py`                                                                 | Memory-aware communication, bandwidth management, sparsification strategies.                           |
| Loop closure node             | Python   | `cslam/loop_closure_detection_node.py`                                             | Descriptor matching and inter-robot loop closure publication.                                          |

### 5.2 Front-End Components

The front-end:

* Synchronizes raw sensor data.
* Generates keyframes.
* Extracts visual or LiDAR descriptors.
* Publishes keyframes and local descriptors.
* Performs or supports loop closure detection.
* Sends validated intra-robot and inter-robot loop closures to the back-end.

### 5.3 Back-End Components

The back-end:

* Maintains local GTSAM pose graph factors and estimates.
* Stores odometry pose estimates.
* Receives loop closure constraints.
* Discovers neighbors through heartbeat and neighbor-manager messages.
* Exchanges pose graphs with neighboring robots.
* Aggregates connected pose graphs.
* Runs robust optimization.
* Publishes optimized estimates.
* Broadcasts TF transforms.

---

## 6. Front-End Architecture

## 6.1 Sensor Handler Interface

```cpp
// File: include/cslam/front_end/sensor_handler_interface.h

class ISensorHandler {
public:
    virtual void process_new_sensor_data() = 0;
};
```

`ISensorHandler` is a pure virtual base class. It provides a common front-end abstraction so that different sensor pipelines can be swapped while preserving the same map-manager and loop-closure architecture.

### 6.2 Sensor Handler Implementations

| Handler             | Language | Sensors                             | Key Features                                                                                                               |
| ------------------- | -------- | ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| **`RGBDHandler`**   | C++      | RGB image + depth image + odometry  | Visual features, depth masking, PnP-based keyframe selection, local image descriptors, optional point cloud visualization. |
| **`StereoHandler`** | C++      | Left/right stereo images + odometry | Disparity-based depth, visual features, stereo callback, inherits / extends RGB-D behavior.                                |
| **`LidarHandler`**  | Python   | `PointCloud2` + odometry            | ICP-based registration, ScanContext descriptors, voxel grid filtering, covariance-based odometry rejection.                |

### 6.3 Sensor Handler Hierarchy

```text
ISensorHandler (interface)
    │
    ├── RGBDHandler
    │   ├── rgbd_callback()             # Receives RGB + depth + odometry
    │   ├── generate_new_keyframe()     # Keyframe selection policy
    │   ├── send_keyframe()             # Publishes to Python node
    │   ├── compute_local_descriptors() # 3D feature extraction
    │   └── send_visualization()        # RViz visualization
    │
    └── StereoHandler (extends RGBDHandler)
        └── stereo_callback()           # Receives left + right images
```

### 6.4 RGB-D Handler

```text
File: src/front_end/rgbd_handler.cpp
```

The `RGBDHandler` processes synchronized RGB-D image data and odometry.

Key details:

* Uses `message_filters::Synchronizer`.
* Uses approximate time synchronization for:

  * RGB image
  * Depth image
  * Camera info
  * Odometry
* Keyframe generation is based on a PnP inlier ratio threshold:

  * `keyframe_generation_ratio_threshold`
* Feature extraction uses RTAB-Map `Feature2D`.
* The feature type may depend on configuration:

  * ORB
  * SIFT
  * AKAZE
* Local descriptors consist of 3D keypoints with descriptors.
* Local descriptors are stored in:

```cpp
local_descriptors_map_
```

* Optional visualization can publish point clouds with voxel subsampling.
* Relevant callbacks / methods:

  * `rgbd_callback()`
  * `generate_new_keyframe()`
  * `send_keyframe()`
  * `compute_local_descriptors()`
  * `send_visualization()`

### 6.5 Stereo Handler

The `StereoHandler` processes synchronized stereo image data and odometry.

Key details:

* Written in C++.
* Handles left and right stereo images.
* Receives odometry.
* Computes depth through stereo / disparity information.
* Extracts visual features.
* Extends the RGB-D handler structure.
* Relevant callback:

```cpp
stereo_callback()
```

### 6.6 LiDAR Handler

```text
File: cslam/lidar_handler_node.py
```

The `LidarHandler` processes LiDAR point clouds and odometry.

Key details:

* Written in Python.
* Uses `ApproximateTimeSynchronizer` for:

  * Point cloud
  * Odometry
* Keyframe generation is distance-based using:

```yaml
keyframe_generation_ratio_distance
```

* Registration is ICP-based.
* ICP utilities are implemented in:

```text
cslam/lidar_pr/icp_utils.py
```

* Filtering includes:

  * Voxel grid downsampling
  * Covariance-based odometry rejection
* LiDAR place recognition can use ScanContext descriptors.
* The LiDAR node generates keyframes and descriptors and participates in loop closure detection.

### 6.7 Map Manager

```cpp
template <class DataHandlerType>
class MapManager : public IMapManager {
    // Manages keyframe queue, loop closure detection pipeline
    // Template parameter: StereoHandler or RGBDHandler
};
```

The `MapManager<DataHandlerType>` is templated so that different front-end data handlers can be reused with the same higher-level keyframe and loop-closure pipeline.

Key responsibilities:

* Receives keyframes from RTAB-Map.
* Manages the keyframe queue.
* Generates keypoints from frames.
* Sends keypoints to other robots.
* Receives keypoints from other robots.
* Computes geometric verification for loop closures.
* Supports loop closure detection pipelines for RGB-D and stereo front-ends.

### 6.8 Loop Closure Detection Pipeline

The front-end detects loop closures in two stages:

1. **Global matching**

   * Compute compact global descriptors for keyframes.
   * Share global descriptors with neighboring robots.
   * Compare descriptors using similarity metrics.
   * Produce candidate loop closures.

2. **Local matching / geometric verification**

   * Request local descriptors or point clouds for selected candidates.
   * Compute geometric registration.
   * Validate loop closures.
   * Publish intra-robot or inter-robot loop closure messages.

### 6.9 Place Recognition Modules

| Module                            | Algorithm       | Purpose                                                      |
| --------------------------------- | --------------- | ------------------------------------------------------------ |
| `lidar_pr/scancontext.py`         | ScanContext     | LiDAR place recognition.                                     |
| `vpr/cosplace.py`                 | Co-SPlace       | Global visual descriptors for image-based place recognition. |
| `vpr/netvlad.py`                  | NetVLAD         | Global visual descriptors.                                   |
| `loop_closure_sparse_matching.py` | Sparse matching | Prioritization / sparse matching support.                    |

### 6.10 Sparse Inter-Robot Loop Closure Candidate Prioritization

Supporting context from the Swarm-SLAM paper describes two prioritization mechanisms after global descriptor matching:

1. **Greedy prioritization**

   * Selects the top `B` candidates with the highest similarity scores.
   * Used in prior work.

2. **Spectral prioritization**

   * Treats candidate selection as a pose graph sparsification problem.
   * Selects candidate inter-robot loop closures by maximizing algebraic connectivity.
   * Uses the global descriptor similarity score as a confidence metric.
   * Aims to reduce redundant geometric verification and communication.
   * Requires the pose graph to be connected; greedy prioritization is used until at least one inter-robot loop closure exists between local pose graphs.
   * Uses the greedy solution as an initial guess for algebraic connectivity maximization. 

The reduced multi-robot graph used for candidate prioritization can be described as:

```text
G = (V, E_local, E_global)

V = (V_1, ..., V_n)

E_local = (E_local_1, ..., E_local_n)

E_global = (E_global_fixed, E_global_candidate)
```

Where:

* `V` contains vertices from all robot pose graphs.
* Each vertex corresponds to a keyframe.
* `E_local` contains local pose graph edges:

  * odometry measurements
  * intra-robot loop closures
* `E_global` contains inter-robot loop closure edges.
* `E_global_fixed` contains already-computed inter-robot measurements.
* `E_global_candidate` contains candidate inter-robot loop closures.
* Candidate edges carry a similarity score.
* Fixed local and global edges can be treated as unweighted edges for prioritization.
* The number of selected edges per step is controlled by a user budget `B`.

### 6.11 Local Feature Exchange as Vertex Cover

For local matching, the system avoids redundant communication by formulating local-feature sharing as a **vertex cover problem**:

* If multiple inter-robot loop closure candidates share a keyframe vertex, only that vertex’s local features need to be transferred once.
* For bipartite graphs, the minimal vertex cover can be computed optimally.
* For three or more robots, the vertex cover is approximated.
* This produces an exchange policy that avoids redundant descriptor or point-cloud transfers.

### 6.12 Temporary Broker for Front-End Communication

For spectral matching and vertex exchange, a temporary broker is dynamically elected among robots in communication range.

Current documented behavior:

* The broker is the robot in range with the lowest ID according to the neighbor management system.
* The broker computes matches.
* The broker sends requests for vertices / local features to be transferred.
* The broker could be replaced by another decentralized mechanism, for example one based on available onboard computation resources.

---

## 7. Back-End Architecture

## 7.1 DecentralizedPGO Class

```text
File: include/cslam/back_end/decentralized_pgo.h
Implementation: src/back_end/decentralized_pgo.cpp
```

```cpp
namespace cslam {
    class DecentralizedPGO : public rclcpp::Node {
        // Main back-end node
    };
}
```

`DecentralizedPGO` is the main back-end node for collaborative pose graph optimization.

It receives:

* Keyframe odometry
* Intra-robot loop closures
* Inter-robot loop closures
* Neighbor information
* Pose graph requests
* Pose graph responses
* Optimization results

It publishes:

* Optimized estimates
* Current optimized pose estimate
* Optimizer state
* Pose graph messages
* Heartbeats
* Reference frame information
* Visualization pose graphs
* Debug optimization results

### 7.2 Key Back-End Responsibilities

1. **Local pose graph**

   * Maintains intra-robot constraints from odometry and loop closures.

2. **Neighbor management**

   * Discovers nearby robots through heartbeat and neighbor-manager messages.

3. **Pose graph exchange**

   * Requests pose graphs from neighboring robots.
   * Receives pose graph responses.
   * Tracks connected robots.

4. **Coordinate alignment**

   * Aligns neighbor graphs to the local or selected origin frame using inter-robot loop closures and anchor/reference-frame logic.

5. **Optimization**

   * Uses GTSAM for robust pose graph optimization.

6. **TF broadcasting**

   * Publishes optimized transforms for integration with the ROS 2 TF tree.

### 7.3 Key Back-End Methods

```cpp
void start_optimization();  // Trigger optimization
void build_pose_graph();    // Construct GTSAM factor graph
void align_pose_graphs();   // Align neighbor graphs to local frame
void optimize();            // Run GTSAM optimizer
void broadcast_results();   // Publish optimized poses
```

Additional important methods:

```cpp
bool is_optimizer();
std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
aggregate_pose_graphs();

std::map<unsigned int, bool> connected_robot_pose_graph();

bool check_waiting_timeout();

void broadcast_tf_callback();
```

### 7.4 GTSAM Integration

Key GTSAM types used:

```cpp
gtsam::NonlinearFactorGraph   // Pose graph factors
gtsam::Values                 // Pose estimates
gtsam::Pose3                  // 3D poses
gtsam::BetweenFactor          // Relative pose constraints
gtsam::GncOptimizer           // Robust optimizer
```

The back-end uses GTSAM to build and optimize nonlinear factor graphs containing:

* Odometry factors
* Intra-robot loop closure factors
* Inter-robot loop closure factors
* Prior / anchor factor

### 7.5 Asynchronous Optimization

Optimization is documented as asynchronous:

* The main optimization loop is driven by timers.
* The heavy optimization step can be executed using `std::async`.
* This prevents the main optimizer state machine from blocking indefinitely during optimization.

---

## 8. Pose Graph Representation

## 8.1 GTSAM Pose Graph

The pose graph is represented using GTSAM.

* **Nodes** are robot keyframe poses.
* **Factors** are relative pose constraints.
* Poses are represented as `gtsam::Pose3`.
* Constraints are represented as `gtsam::BetweenFactor<gtsam::Pose3>`.

### 8.2 GTSAM Key Schema

Nodes use `gtsam::LabeledSymbol` with a hierarchical structure:

```text
LabeledSymbol = (GRAPH_LABEL, ROBOT_LABEL(robot_id), keyframe_id)
├── GRAPH_LABEL = 'g'              // Graph label
├── ROBOT_LABEL(id) = 'A' + id     // Robot label: A, B, C, ...
└── keyframe_id = uint32           // Keyframe ID

Example: ('g', 'A', 42) → keyframe 42 of robot 0
```

### 8.3 Factor Types

| Factor Type                   | Description                                          | Source                                   |
| ----------------------------- | ---------------------------------------------------- | ---------------------------------------- |
| **Odometry factors**          | Constraints between consecutive keyframes.           | `odometry_callback()`                    |
| **Intra-robot loop closures** | Constraints between keyframes from the same robot.   | `intra_robot_loop_closure_callback()`    |
| **Inter-robot loop closures** | Constraints between keyframes from different robots. | `inter_robot_loop_closure_callback()`    |
| **Prior / anchor factor**     | Fixes the reference frame for optimization.          | Added during aggregation / optimization. |

### 8.4 Noise Model

```cpp
default_noise_model_ = gtsam::noiseModel::Diagonal::Variances(
    {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});  // 6 DOF: x, y, z, roll, pitch, yaw
```

### 8.5 Pose Graph Values and Edges

Pose graph exchange uses custom messages that represent GTSAM values and factors.

#### `PoseGraphValue`

```text
uint32 robot_id          # ID of the robot
uint32 keyframe_id       # ID of the keyframe
float64[7] pose          # [x, y, z, qw, qx, qy, qz]
```

#### `PoseGraphEdge`

```text
uint32 robot0_id         # Source robot
uint32 keyframe0_id      # Source keyframe
uint32 robot1_id         # Destination robot
uint32 keyframe1_id      # Destination keyframe
float64[7] transform     # Relative transform
float64[6] covariance    # 6x6 covariance, diagonal only
```

### 8.6 Multi-Robot Key

`MultiRobotKey` represents a pair:

```text
robot_id + keyframe_id
```

It is used to identify a pose/keyframe in a multi-robot pose graph.

---

## 9. Optimization State Machine

## 9.1 Optimizer State Enumeration

```cpp
enum class OptimizerState {
    IDLE,                             // 0: Inactive, waiting for trigger
    WAITING_FOR_NEIGHBORS_INFO,       // 1: Waiting for neighbor information
    POSEGRAPH_COLLECTION,             // 2: Collecting pose graphs from neighbors
    WAITING_FOR_NEIGHBORS_POSEGRAPHS, // 3: Waiting for pose graph responses
    START_OPTIMIZATION,               // 4: Ready to start optimization
    OPTIMIZATION                      // 5: Optimization running asynchronously
};
```

### 9.2 State Machine Diagram

```text
                    ┌─────────────────────────────────────────────┐
                    │                                            │
                    ▼                                            │
              ┌──────────┐                                      │
              │   IDLE   │                                      │
              └────┬─────┘                                      │
                   │ odometry received                           │
                   ▼                                             │
    ┌──────────────────────────┐                                  │
    │ WAITING_FOR_NEIGHBORS_   │                                  │
    │         INFO             │                                  │
    └──────────┬───────────────┘                                  │
               │ neighbors info received                          │
               ▼                                                  │
    ┌──────────────────────────┐     ┌───────────┐                │
    │   POSEGRAPH_COLLECTION   │────>│   IDLE    │ no neighbors   │
    └──────────┬───────────────┘     └───────────┘                │
               │ request pose graphs                              │
               ▼                                                  │
    ┌───────────────────────────────┐                              │
    │ WAITING_FOR_NEIGHBORS_        │                              │
    │       POSEGRAPHS              │                              │
    └──────────┬────────────────────┘                              │
               │ all graphs received / timeout                     │
               ▼                                                  │
    ┌──────────────────────────┐                                  │
    │   START_OPTIMIZATION     │                                  │
    └──────────┬───────────────┘                                  │
               │ start optimization                               │
               ▼                                                  │
    ┌──────────────────────────┐                                  │
    │      OPTIMIZATION        │                                  │
    └──────────┬───────────────┘                                  │
               │ result ready                                     │
               ▼                                                  │
              ┌──────────┐                                       │
              │   IDLE   │                                       │
              └──────────┘                                       │
```

A simplified state sequence is:

```text
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
  ↓
IDLE
```

### 9.3 State Transition Conditions

| State                              | Trigger / Exit Condition                                                     |
| ---------------------------------- | ---------------------------------------------------------------------------- |
| `IDLE`                             | Optimization trigger and odometry available.                                 |
| `WAITING_FOR_NEIGHBORS_INFO`       | Neighbor information received.                                               |
| `POSEGRAPH_COLLECTION`             | Pose graph requests sent, or return to `IDLE` if no neighbors are available. |
| `WAITING_FOR_NEIGHBORS_POSEGRAPHS` | All requested graphs received or timeout occurs.                             |
| `START_OPTIMIZATION`               | Aggregated graph is ready.                                                   |
| `OPTIMIZATION`                     | Async optimization finishes and results are published.                       |

### 9.4 Optimizer Selection

The method `is_optimizer()` determines whether the current robot should execute the optimization:

```cpp
bool DecentralizedPGO::is_optimizer()
{
    // Priority based on the robot ID and its origin
    bool is_optimizer = true;
    for (unsigned int i = 0; i < current_neighbors_ids_.origins.ids.size(); i++)
    {
        if (origin_robot_id_ > current_neighbors_ids_.origins.ids[i])
        {
            is_optimizer = false;  // Another robot has higher priority
        }
        else if (origin_robot_id_ == current_neighbors_ids_.origins.ids[i] &&
                 robot_id_ > current_neighbors_ids_.robots.ids[i])
        {
            is_optimizer = false;  // Same origin, but larger robot ID
        }
    }
    // Must have at least one odometry pose
    if (odometry_pose_estimates_->size() == 0)
    {
        is_optimizer = false;
    }
    return is_optimizer;
}
```

Priority rule from the detailed back-end document:

1. The robot with the lowest `origin_robot_id` becomes the optimizer.
2. If multiple robots share the same `origin_robot_id`, the robot with the lowest `robot_id` wins.
3. A robot cannot be optimizer if it has no odometry pose estimates.

A separate high-level summary states the default selection as:

```text
Robot with the lowest ID becomes the optimizer.
```

This difference is preserved in **Section 24: Potential Inconsistencies / Items to Verify**.

---

## 10. Optimization Pipeline

## 10.1 High-Level Optimization Workflow

1. **Local map building**

   * Each robot builds its local pose graph from odometry and intra-robot loop closures.

2. **Neighbor discovery**

   * Robots broadcast heartbeat messages and discover nearby robots.

3. **Descriptor exchange**

   * Robots exchange global and local descriptors for cross-robot matching.

4. **Inter-robot loop closure detection**

   * Valid cross-robot matches become inter-robot loop closure constraints.

5. **Pose graph exchange**

   * Robots request and receive pose graphs from neighbors.

6. **Coordinate alignment**

   * Neighbor pose graphs are aligned to the selected local or global reference frame.

7. **Joint optimization**

   * GTSAM optimizes the aggregated connected pose graph.

8. **Result distribution**

   * Optimized poses are published back to participating robots.

9. **TF publishing**

   * Optimized TF frames are broadcast.

### 10.2 Detailed Optimization Phases

```text
┌─────────────────────────────────────────────────────────────────┐
│  Phase 1: Data Collection                                       │
│  ────────────────────────                                       │
│  - Receive odometry (keyframe_odom)                             │
│  - Receive loop closures (intra + inter robot)                  │
│  - Build local pose graph                                       │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Phase 2: Communication                                         │
│  ──────────────────────                                         │
│  - Request current neighbors                                    │
│  - Exchange pose graphs with neighbors                          │
│  - Check graph connectivity with BFS                            │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Phase 3: Aggregation                                           │
│  ───────────────────                                            │
│  - Merge all received pose graphs                               │
│  - Filter by connectivity                                       │
│  - Add prior on first pose                                      │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Phase 4: Optimization                                          │
│  ───────────────────                                            │
│  - GNC optimizer with Levenberg-Marquardt                       │
│  - Asynchronous execution with std::async                       │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Phase 5: Distribution                                          │
│  ─────────────────────                                          │
│  - Publish results on /r{ID}/cslam/optimized_estimates          │
│  - Update TF frames                                             │
│  - Logging, if enabled                                          │
└─────────────────────────────────────────────────────────────────┘
```

### 10.3 Pose Graph Aggregation

`aggregate_pose_graphs()` performs:

1. Connectivity verification through BFS.
2. Aggregation of local graph factors.
3. Aggregation of local odometry estimates.
4. Aggregation of estimates from connected neighboring robots.
5. Aggregation of local inter-robot factors.
6. Aggregation of neighbor factors.
7. Deduplication of loop closures using `std::set`.

```cpp
std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
DecentralizedPGO::aggregate_pose_graphs()
{
    // 1. Check connectivity
    auto is_pose_graph_connected = connected_robot_pose_graph();

    // 2. Local graph + odometry estimates
    graph->push_back(pose_graph_->begin(), pose_graph_->end());
    estimates->insert(*odometry_pose_estimates_);

    // 3. Estimates from connected neighbors
    for (auto id : current_neighbors_ids_.robots.ids) {
        if (is_pose_graph_connected[id]) {
            estimates->insert(*other_robots_graph_and_estimates_[id].second);
        }
    }

    // 4. Local inter-robot factors
    // 5. Neighbor graph factors
    // ... with deduplication
}
```

### 10.4 Connectivity Verification

The method `connected_robot_pose_graph()` performs a breadth-first search:

```cpp
std::map<unsigned int, bool> DecentralizedPGO::connected_robot_pose_graph()
{
    // BFS from the current robot
    // Visit every robot reachable through inter-robot loop closures
    // Return map {robot_id: is_connected}
}
```

Only robots connected to the current collaborative pose graph component contribute to global optimization.

This allows the system to work with partial network topologies and sporadic robot rendezvous.

### 10.5 Optimization Algorithm

```cpp
gtsam::Values DecentralizedPGO::optimize(
    const gtsam::NonlinearFactorGraph::shared_ptr &graph,
    const gtsam::Values::shared_ptr &initial)
{
    // GNC = Generalized Concave / Graduated Non-Convexity depending on source wording
    // Uses Levenberg-Marquardt as internal solver
    gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
    gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
        optimizer(*graph, *initial, params);
    return optimizer.optimize();
}
```

The optimizer is documented as a GTSAM **GNC optimizer** with Levenberg-Marquardt parameters.

The supporting Swarm-SLAM paper describes the back-end as using **Graduated Non-Convexity** with a robust **Truncated Least Square** loss. 

### 10.6 Result Distribution

After optimization:

* Optimized values are converted to ROS messages.
* Results are filtered per robot.
* Each robot receives its relevant optimized estimates.
* Results are published on:

```text
/r{ID}/cslam/optimized_estimates
```

* The local robot applies received optimized estimates in:

```cpp
optimized_estimates_callback()
```

* TF frames are updated.
* Logs and debug topics may be published.

---

## 11. Communication Architecture

## 11.1 Communication Model

The system is decentralized:

* No permanent central server is required.
* Robots communicate peer-to-peer.
* Robots may only communicate during rendezvous.
* Neighbor management handles dynamic connectivity.
* A temporary broker or elected optimizer may exist during a rendezvous, but this is not a permanent central authority.

### 11.2 Neighbor Management

```python
# neighbors_manager.py
class NeighborsManager:
    # Discovers neighbors via Zenoh
    # Maintains list of connected robots
    # Publishes current neighbors to back-end
```

Responsibilities:

* Publish heartbeat messages.
* Detect robots in communication range.
* Maintain current neighbor lists.
* Track exchanged measurements to avoid redundant bandwidth usage.
* Publish neighbor information to the back-end.
* Support sporadic communication and ad-hoc networks.

### 11.3 Heartbeat

Robots publish heartbeat messages at a fixed rate.

Relevant parameter:

```yaml
heartbeat_period_sec: 1.0
```

Relevant topic:

```text
cslam/heartbeat
```

Message type:

```text
UInt32
```

### 11.4 Zenoh Transport

The system uses Zenoh for inter-robot communication.

Zenoh provides:

* Multicast support for neighbor discovery.
* Reliable message delivery.
* Low-latency communication.
* Ad-hoc networking support.
* A transport layer for pose graph exchange, descriptor sharing, and neighbor discovery.

### 11.5 Memory-Aware Communication

```python
# mac/mac.py
# Manages communication bandwidth
# Prioritizes critical messages
# Implements sparsification strategies
```

Responsibilities:

* Manage bandwidth.
* Prioritize critical messages.
* Support sparsification strategies.
* Reduce unnecessary communication.

### 11.6 Pose Graph Exchange Protocol

```text
┌──────────────────────────────────────────────────────────────┐
│  Pose Graph Request                                          │
│  ──────────────────                                          │
│                                                              │
│  1. Optimizer sends "get_pose_graph" to each neighbor         │
│     Topic: /r{ID}/cslam/get_pose_graph                       │
│     Payload: RobotIds (list of all involved robots)           │
│                                                              │
│  2. Each neighbor responds with its own pose graph            │
│     Topic: /cslam/pose_graph                                 │
│     Payload: PoseGraph (edges + values + connected_robots)    │
│                                                              │
│  3. Optimizer aggregates all received graphs                  │
│     - Checks connectivity with BFS                            │
│     - Merges factors and values                               │
│     - Deduplicates constraints                                │
│                                                              │
│  4. Results are distributed to all robots                     │
│     Topic: /r{ID}/cslam/optimized_estimates                   │
│     Payload: OptimizationResult (estimates filtered by robot) │
└──────────────────────────────────────────────────────────────┘
```

### 11.7 Back-End Communication Sequence

```text
Robot A (Optimizer)          Robot B (Neighbor)         Robot C (Neighbor)
      │                          │                          │
      │  get_current_neighbors   │                          │
      │─────────────────────────>│                          │
      │                          │                          │
      │  current_neighbors       │                          │
      │<─────────────────────────│                          │
      │                          │                          │
      │  get_pose_graph          │                          │
      │─────────────────────────>│                          │
      │                          │                          │
      │  pose_graph              │                          │
      │<─────────────────────────│                          │
      │                          │                          │
      │         [Aggregate local pose graphs]                │
      │         [Add prior on first pose]                    │
      │         [Run GNC Optimizer with LM]                  │
      │                          │                          │
      │  optimized_estimates     │  optimized_estimates     │
      │─────────────────────────>│─────────────────────────>│
      │                          │                          │
      │  [Apply estimates,       │  [Apply estimates,       │
      │   update TF]             │   update TF]             │
      │                          │                          │
```

### 11.8 Timeout Handling

```cpp
bool DecentralizedPGO::check_waiting_timeout()
{
    if ((node_->now() - start_waiting_time_) > max_waiting_time_sec_)
    {
        end_waiting();
        optimizer_state_ = OptimizerState::IDLE;
        RCLCPP_INFO(node_->get_logger(), "Timeout: (%d)", robot_id_);
    }
    return is_waiting();
}
```

Timeout behavior:

* Prevents deadlock when neighbors do not respond.
* Ends waiting state.
* Resets optimizer state to `IDLE`.
* Logs timeout information.

---

## 12. ROS 2 Topics

## 12.1 DecentralizedPGO Input Topics

| Topic                            | Message Type                      | Purpose                                                   |
| -------------------------------- | --------------------------------- | --------------------------------------------------------- |
| `cslam/keyframe_odom`            | `KeyframeOdom`                    | Receives keyframe odometry and builds odometry factors.   |
| `cslam/intra_robot_loop_closure` | `IntraRobotLoopClosure`           | Receives loop closures within the same robot.             |
| `cslam/inter_robot_loop_closure` | `InterRobotLoopClosure`           | Receives loop closures between different robots.          |
| `cslam/optimized_estimates`      | `OptimizationResult`              | Applies optimization results.                             |
| `cslam/get_pose_graph`           | `RobotIds`                        | Receives pose graph requests.                             |
| `/cslam/pose_graph`              | `PoseGraph`                       | Receives pose graphs from neighbors.                      |
| `cslam/current_neighbors`        | `RobotIdsAndOrigin`               | Receives current neighbor list.                           |
| `cslam/write_current_estimates`  | Not specified in source documents | Triggers writing current graph / estimates in G2O format. |

### 12.2 DecentralizedPGO Output Topics

| Topic                                | Message Type         | Purpose                                    |
| ------------------------------------ | -------------------- | ------------------------------------------ |
| `/r{ID}/cslam/optimized_estimates`   | `OptimizationResult` | Sends optimized estimates to robot `{ID}`. |
| `/r{ID}/cslam/current_pose_estimate` | `PoseStamped`        | Publishes current optimized pose estimate. |
| `cslam/optimizer_state`              | `OptimizerState`     | Publishes optimizer state.                 |
| `cslam/get_current_neighbors`        | `String`             | Requests current neighbors.                |
| `/r{ID}/cslam/get_pose_graph`        | `RobotIds`           | Requests pose graph from robot `{ID}`.     |
| `/cslam/pose_graph`                  | `PoseGraph`          | Publishes local pose graph response.       |
| `/cslam/viz/pose_graph`              | `PoseGraph`          | Publishes pose graph for visualization.    |
| `cslam/heartbeat`                    | `UInt32`             | Publishes heartbeat for discovery.         |
| `cslam/reference_frames`             | `ReferenceFrames`    | Publishes reference-frame information.     |
| `cslam/debug_optimization_result`    | `OptimizationResult` | Publishes debug optimization result.       |

### 12.3 TF Broadcasts

```text
map_{origin} → map_{local}
map_{origin} → latest_optimized_{local}
latest_optimized_{local} → current_{local}
```

### 12.4 Front-End Topics and Expected Back-End Inputs

The back-end expects the front-end to publish:

| Topic                            | Message Type            |
| -------------------------------- | ----------------------- |
| `cslam/keyframe_odom`            | `KeyframeOdom`          |
| `cslam/intra_robot_loop_closure` | `IntraRobotLoopClosure` |
| `cslam/inter_robot_loop_closure` | `InterRobotLoopClosure` |

Front-end sensor topics are configurable. Example parameters:

```yaml
color_image_topic: "color/image"
depth_image_topic: "depth/image"
odom_topic: "odom"
```

### 12.5 Topic Diagram

```text
┌─────────────────────────────────────────────────────────────────────┐
│                     DecentralizedPGO Node                           │
│                                                                     │
│  INPUT TOPICS:                                                      │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │ cslam/keyframe_odom              (KeyframeOdom)           │      │
│  │ cslam/intra_robot_loop_closure   (IntraRobotLoopClosure)  │      │
│  │ cslam/inter_robot_loop_closure   (InterRobotLoopClosure)  │      │
│  │ cslam/optimized_estimates        (OptimizationResult)     │      │
│  │ cslam/get_pose_graph             (RobotIds)               │      │
│  │ /cslam/pose_graph                (PoseGraph)              │      │
│  │ cslam/current_neighbors          (RobotIdsAndOrigin)      │      │
│  └───────────────────────────────────────────────────────────┘      │
│                                                                     │
│  OUTPUT TOPICS:                                                     │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │ /r{ID}/cslam/optimized_estimates   (OptimizationResult)   │      │
│  │ /r{ID}/cslam/current_pose_estimate  (PoseStamped)         │      │
│  │ cslam/optimizer_state              (OptimizerState)       │      │
│  │ cslam/get_current_neighbors        (String)               │      │
│  │ /r{ID}/cslam/get_pose_graph        (RobotIds)             │      │
│  │ /cslam/pose_graph                  (PoseGraph)            │      │
│  │ /cslam/viz/pose_graph              (PoseGraph)            │      │
│  │ cslam/heartbeat                    (UInt32)               │      │
│  │ cslam/reference_frames             (ReferenceFrames)      │      │
│  │ cslam/debug_optimization_result    (OptimizationResult)   │      │
│  └───────────────────────────────────────────────────────────┘      │
│                                                                     │
│  TF BROADCAST:                                                      │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │ map_{origin} → map_{local}                                │      │
│  │ map_{origin} → latest_optimized_{local}                   │      │
│  │ latest_optimized_{local} → current_{local}                │      │
│  └───────────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 13. Custom Message Interfaces

## 13.1 Message Packages

Custom interfaces are described under:

```text
cslam_common_interfaces
```

and also in the repository tree as:

```text
src/cslam_interfaces/
├── cslam_common_interfaces/
└── cslam_zenoh_interfaces/
```

The `cslam_zenoh_interfaces` package contains Zenoh-specific messages.

### 13.2 Common Interfaces

| Message                      | Purpose                                                      |
| ---------------------------- | ------------------------------------------------------------ |
| `KeyframeRGB`                | RGB keyframe data for loop detection.                        |
| `KeyframePointCloud`         | Point cloud keyframe data for loop detection.                |
| `KeyframeOdom`               | Odometry with keyframe ID.                                   |
| `LocalImageDescriptors`      | Local image descriptors for matching.                        |
| `LocalPointCloudDescriptors` | Local point cloud descriptors for matching.                  |
| `LocalDescriptorsRequest`    | Request descriptors from another robot.                      |
| `LocalKeyframeMatch`         | Local match result between robots.                           |
| `InterRobotLoopClosure`      | Loop closure between two different robots.                   |
| `IntraRobotLoopClosure`      | Loop closure within the same robot.                          |
| `PoseGraph`                  | Complete pose graph with edges and values.                   |
| `PoseGraphEdge`              | Edge between two poses / `MultiRobotKey` entries.            |
| `PoseGraphValue`             | Pose estimate for a keyframe.                                |
| `OptimizationResult`         | Optimized pose estimates.                                    |
| `RobotIds`                   | List of robot IDs.                                           |
| `RobotIdsAndOrigin`          | Neighbor discovery message containing robot IDs and origins. |
| `VizPointCloud`              | Visualization point clouds.                                  |
| `GlobalDescriptor`           | Global place recognition descriptor.                         |
| `MultiRobotKey`              | Robot ID + keyframe ID pair.                                 |
| `OptimizerState`             | Current optimizer state.                                     |
| `ReferenceFrames`            | Reference-frame information.                                 |

### 13.3 Standard ROS Message Types Used

| Message Type                      | Usage                                            |
| --------------------------------- | ------------------------------------------------ |
| `nav_msgs::msg::Odometry`         | Odometry input and conversion to `gtsam::Pose3`. |
| `geometry_msgs::msg::Transform`   | Transform input / output conversion.             |
| `geometry_msgs::msg::Pose`        | Pose output conversion.                          |
| `geometry_msgs::msg::PoseStamped` | Current pose estimate publication.               |
| `sensor_msgs::msg::Image`         | RGB, depth, stereo image streams.                |
| `sensor_msgs::msg::PointCloud2`   | LiDAR and visualization point clouds.            |
| `std_msgs::msg::String`           | Neighbor request topic.                          |
| `std_msgs::msg::UInt32`           | Heartbeat topic.                                 |

### 13.4 GTSAM ↔ ROS Conversion Utilities

| Function                        | Direction   | Type                                             |
| ------------------------------- | ----------- | ------------------------------------------------ |
| `odometry_msg_to_pose3()`       | ROS → GTSAM | `nav_msgs::msg::Odometry` → `gtsam::Pose3`       |
| `transform_msg_to_pose3()`      | ROS → GTSAM | `geometry_msgs::msg::Transform` → `gtsam::Pose3` |
| `gtsam_pose_to_msg()`           | GTSAM → ROS | `gtsam::Pose3` → `geometry_msgs::msg::Pose`      |
| `gtsam_pose_to_transform_msg()` | GTSAM → ROS | `gtsam::Pose3` → `geometry_msgs::msg::Transform` |
| `gtsam_values_to_msg()`         | GTSAM → ROS | `gtsam::Values` → `vector<PoseGraphValue>`       |
| `values_msg_to_gtsam()`         | ROS → GTSAM | `vector<PoseGraphValue>` → `gtsam::Values`       |
| `gtsam_factors_to_msg()`        | GTSAM → ROS | `NonlinearFactorGraph` → `vector<PoseGraphEdge>` |
| `edges_msg_to_gtsam()`          | ROS → GTSAM | `vector<PoseGraphEdge>` → `NonlinearFactorGraph` |

---

## 14. Multi-Robot Collaboration Workflow

## 14.1 Multi-Robot Collaboration Diagram

```text
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

### 14.2 Decentralized PGO Workflow

1. Each robot maintains its own local pose graph.
2. Robots periodically exchange pose graphs with neighbors.
3. An optimizer is selected.
4. The optimizer aggregates all connected graphs.
5. The optimizer runs GTSAM optimization.
6. Results are shared with participating robots.
7. Each robot applies its optimized estimates.
8. TF frames are updated.

### 14.3 Loop Closure Detection Workflow

1. Global descriptors are computed for each keyframe.
2. Global descriptors are shared with neighboring robots.
3. Matching is performed using similarity metrics.
4. Candidate matches are optionally prioritized by greedy or spectral sparsification.
5. Local descriptors or point clouds are requested for selected candidates.
6. Geometric verification validates candidate loop closures.
7. Valid loop closures are published as:

   * `IntraRobotLoopClosure`
   * `InterRobotLoopClosure`
8. The back-end adds loop closure constraints to the pose graph.

### 14.4 ScanContext LiDAR Workflow

1. LiDAR scans are converted to ScanContext representations.
2. Ring and sector-based features are extracted.
3. Fast matching uses contextual information.
4. ICP refinement computes precise alignment.
5. Valid LiDAR loop closures are sent to the pose graph.

### 14.5 Rendezvous-Based Collaboration

A rendezvous occurs when a subset of robots are within communication range.

During rendezvous:

* Neighbor manager detects current neighbors.
* Robots exchange descriptors or pose graphs.
* A broker or optimizer may be temporarily elected.
* Inter-robot loop closures can be computed.
* Pose graphs are aggregated and optimized.
* Reference-frame information can propagate across the robot team.

The supporting paper describes convergence to a global reference frame through successive rendezvous between subsets of robots rather than requiring all robots to meet simultaneously. 

---

## 15. TF Frames and Coordinate Alignment

## 15.1 TF Frame Hierarchy

```text
map_{origin_robot}                     ← Global reference frame
    │
    ├── map_{robot_0}                  ← Robot 0 map frame
    │   ├── latest_optimized_{robot_0} ← Last optimized pose
    │   │   └── current_{robot_0}      ← Current odometry pose
    │
    ├── map_{robot_1}
    │   ├── latest_optimized_{robot_1}
    │   │   └── current_{robot_1}
    │
    └── map_{robot_N}
        ├── latest_optimized_{robot_N}
        │   └── current_{robot_N}
```

### 15.2 Frame ID Macros

```cpp
#define MAP_FRAME_ID(id)              "map_" + std::to_string(id)
#define LATEST_OPTIMIZED_FRAME_ID(id) "latest_optimized_" + std::to_string(id)
#define CURRENT_FRAME_ID(id)          "current_" + std::to_string(id)
```

### 15.3 Broadcasted Transforms

`broadcast_tf_callback()` publishes three transforms:

1. `map_{origin} → map_{local}`
2. `map_{origin} → latest_optimized_{local}`
3. `latest_optimized_{local} → current_{local}`

```cpp
void DecentralizedPGO::broadcast_tf_callback()
{
    // 1. Origin → Local map
    tf_broadcaster_->sendTransform(origin_to_first_pose_);

    // 2. Origin → Latest optimized pose
    tf_broadcaster_->sendTransform(latest_optimized_pose_msg);

    // 3. Latest optimized → Current (odometry delta)
    gtsam::Pose3 current_pose_diff =
        local_pose_at_latest_optimization_.inverse() * latest_local_pose_;
    tf_broadcaster_->sendTransform(current_transform_msg);

    // Publish current pose estimate (optimized + odometry)
    optimized_pose_estimate_publisher_->publish(pose_msg);
}
```

### 15.4 Anchor and Reference Frame Selection

The supporting paper describes an anchor-selection process:

* At the start, each robot is in its own local reference frame.
* The origin corresponds to that robot’s first pose.
* During a first rendezvous, the first pose of the robot with the lowest ID can become the anchor.
* In later rendezvous, the anchor is selected based on the lowest current reference-frame ID.
* This allows the global reference frame to propagate through the team over multiple sporadic rendezvous.
* Robots can converge to a single global reference frame without all robots being connected simultaneously. 

### 15.5 Coordinate Alignment Responsibilities

Coordinate alignment is handled by the back-end through:

```cpp
void align_pose_graphs();
```

and through TF publication.

The system aligns different robot coordinate frames using:

* Inter-robot loop closures
* Origin / reference frame information
* Anchor / prior selection
* Optimized pose graph estimates

---

## 16. Configuration

## 16.1 Front-End Parameters

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

### 16.2 Back-End Parameters

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

### 16.3 Visualization Parameters

```yaml
visualization:
  enable: false
  publishing_period_ms: 100
  voxel_size: 0.05
  max_range: 10.0
```

### 16.4 Sensor Configuration Example

```yaml
# Example: kitti_stereo.yaml
robot_id: 0
max_nb_robots: 4
sensor_type: stereo
keyframe_period: 0.5
loop_closure_detection_period: 1.0
optimization_period: 5000  # ms
```

### 16.5 Main Parameter Table

| Parameter                                 |       Type |            Default | Description                                                      |
| ----------------------------------------- | ---------: | -----------------: | ---------------------------------------------------------------- |
| `robot_id`                                |   `uint32` |                `-` | Unique robot ID.                                                 |
| `max_nb_robots`                           |   `uint32` |                `-` | Maximum number of robots in the system.                          |
| `sensor_type`                             |   `string` |                `-` | Sensor type, for example `stereo`, RGB-D, or LiDAR.              |
| `keyframe_period`                         |    numeric |                `-` | Keyframe generation period.                                      |
| `loop_closure_detection_period`           |    numeric |                `-` | Loop closure detection period.                                   |
| `optimization_period`                     |    numeric |                `-` | Optimization period in milliseconds, from sensor config example. |
| `pose_graph_optimization_start_period_ms` |   `uint32` |                `-` | Period for optimization trigger.                                 |
| `pose_graph_optimization_loop_period_ms`  |   `uint32` |                `-` | Main optimizer loop period.                                      |
| `max_waiting_time_sec`                    | `Duration` |                `-` | Timeout while waiting for neighbors.                             |
| `heartbeat_period_sec`                    | `Duration` |                `-` | Heartbeat period.                                                |
| `enable_broadcast_tf_frames`              |     `bool` |                `-` | Enables TF broadcasting.                                         |
| `enable_pose_timestamps_recording`        |     `bool` |                `-` | Records pose timestamps.                                         |
| `enable_gps_recording`                    |     `bool` |                `-` | Records GPS data.                                                |
| `enable_simulated_rendezvous`             |     `bool` |                `-` | Enables simulated rendezvous.                                    |
| `enable_logs`                             |     `bool` |                `-` | Enables detailed logging.                                        |
| `log_folder`                              |   `string` |                `-` | Folder for log output.                                           |
| `max_queue_size`                          |    integer |   `100` in example | Front-end queue size.                                            |
| `keyframe_generation_ratio_threshold`     |      float |   `0.5` in example | RGB-D / stereo PnP keyframe threshold.                           |
| `keyframe_generation_ratio_distance`      |      float |   `0.5` in example | LiDAR distance-based keyframe threshold.                         |
| `pnp_min_inliers`                         |    integer |    `20` in example | Minimum PnP inliers.                                             |
| `color_image_topic`                       |     string |    `"color/image"` | RGB image topic.                                                 |
| `depth_image_topic`                       |     string |    `"depth/image"` | Depth image topic.                                               |
| `odom_topic`                              |     string |           `"odom"` | Odometry topic.                                                  |
| `visualization.enable`                    |       bool | `false` in example | Enables visualization output.                                    |
| `visualization.publishing_period_ms`      |    integer |   `100` in example | Visualization publishing period.                                 |
| `visualization.voxel_size`                |      float |  `0.05` in example | Visualization voxel size.                                        |
| `visualization.max_range`                 |      float |  `10.0` in example | Visualization maximum range.                                     |

### 16.6 Configuration Files

Documented configuration files and directories:

```text
config/params.yaml
config/cslam/example.yaml
config/rendezvous/
config/s3e/
config/kitti_stereo.yaml
config/kitti_lidar.yaml
config/realsense_rgbd.yaml
config/ouster_lidar.yaml
```

### 16.7 Simulated Rendezvous

```cpp
// simulated_rendezvous.h
class SimulatedRendezVous {
    // Reads schedule file
    // Determines when robots are "alive"
    // Simulates network connectivity
};
```

---

## 17. Build and Run

## 17.1 Build

```bash
colcon build --packages-select cslam cslam_interfaces cslam_zenoh
source install/setup.bash
```

### 17.2 Run Example

```bash
# Launch with Zenoh transport
ros2 launch cslam multi_robot_slam.launch.py \
    sensor_type:=stereo \
    config_file:=kitti_stereo.yaml \
    robot_id:=0
```

### 17.3 Runtime Organization

At runtime, each robot runs:

* A front-end sensor handler / map manager.
* Loop closure detection components.
* Neighbor manager.
* Communication layer.
* Back-end `DecentralizedPGO`.
* Optional visualization and logging.

---

## 18. Dependencies

## 18.1 Main Dependencies

| Dependency          | Purpose                                                                      |
| ------------------- | ---------------------------------------------------------------------------- |
| **ROS 2**           | Node management, communication, TF, timers, launch.                          |
| **GTSAM**           | Pose graph optimization and nonlinear factor graphs.                         |
| **RTAB-Map**        | Feature extraction, visual odometry support, loop detection, SLAM utilities. |
| **OpenCV**          | Image processing and feature detection.                                      |
| **PCL**             | Point cloud processing.                                                      |
| **Eigen**           | Linear algebra.                                                              |
| **Boost**           | Smart pointers and graph algorithms.                                         |
| **cv_bridge**       | ROS ↔ OpenCV image conversion.                                               |
| **tf2_ros**         | Coordinate transform management and TF broadcasting.                         |
| **message_filters** | Approximate synchronization of multiple sensor streams.                      |
| **image_transport** | ROS image transport.                                                         |
| **Zenoh**           | Inter-robot communication transport.                                         |

### 18.2 C++ Dependencies

* GTSAM
* RTAB-Map
* OpenCV
* Eigen
* Boost
* PCL
* ROS 2 `rclcpp`
* `tf2_ros`
* `message_filters`
* `image_transport`
* `cv_bridge`

### 18.3 Python Dependencies

* NumPy
* PyTorch
* Open3D
* Zenoh-Python
* ROS 2 `rclpy`

### 18.4 ROS 2 Message Dependencies

* `nav_msgs`
* `sensor_msgs`
* `geometry_msgs`
* `std_msgs`
* `cslam_common_interfaces`
* `cslam_zenoh_interfaces`

---

## 19. Experiments

## 19.1 Experiment Framework

The repository contains experiment configuration directories:

```text
experiments/
├── datasets_experiments/
├── robot_experiments/
├── odometry/
└── sensors/
```

Experiment categories include:

* Dataset experiments
* Robot experiments
* Odometry benchmarks
* Sensor comparisons

Documented datasets and configurations include:

* KITTI
* EuRoC
* S3E
* KITTI stereo
* KITTI LiDAR
* RealSense RGB-D
* Ouster LiDAR

### 19.2 Swarm-SLAM Paper Evaluation Context

The supporting Swarm-SLAM paper reports evaluation on multiple public datasets and one real-world deployment.

Dataset examples from the paper include:

* KITTI 00 stereo sequence
* KITTI360 09 LiDAR sequence
* GrAco
* M2DGR
* S3E

The real-world experiment involved:

* Three robots:

  * Boston Dynamics Spot
  * Agilex Scout
  * Agilex Scout Mini
* NVIDIA Jetson AGX Xavier onboard computers
* Intel RealSense D455 cameras
* Ouster OS0-64 LiDARs
* VectorNav VN100 IMUs
* GL-iNet GL-S1300 OpenWrt gateways
* Ad-hoc networking
* LiDAR + IMU odometry
* RGB-D cameras for inter-robot loop closure detection

Reported statistics include:

* 475 meters traveled
* 3103 keyframes
* 67 loop closures
* 10 loop closures rejected by GNC
* 94.95 MB transmitted between robots, excluding visualization
* Sparsification and optimization executed in separate threads because sparsification time was non-negligible but lower than pose graph optimization time 

### 19.3 Experiment Metrics

The system supports or reports:

* Absolute Translation Error (ATE)
* Pose graph statistics
* Optimization time
* Communication overhead
* Loop closure success rates
* Loop closure failure rates
* GPS-based error metrics, when available
* Pose timestamps
* Initial and optimized pose graph logs

### 19.4 Evaluation Mode

Evaluation mode enables:

* GPS recording
* Pose timestamp recording
* Logging for benchmarking
* Pose graph export
* Optimization timing analysis

---

## 20. Logging, Debugging, and Metrics

## 20.1 Debug Pipeline Logs

The back-end includes debug logs using the tag:

```text
[DEBUG_BACKEND_PIPELINE]
```

Example:

```cpp
RCLCPP_INFO(node_->get_logger(),
    "[DEBUG_BACKEND_PIPELINE] keyframe_odom received count=%lu id=%u ...",
    odom_cb_count, msg->id, ...);
```

### 20.2 Logger

If `enable_logs_` is active, the `Logger` records:

* Pose timestamps
* Initial pose graph
* Optimized pose graph
* Information about received graphs
* Optimization times
* Pose graph statistics
* Communication overhead
* Loop closure success / failure rates
* GPS-based error metrics, when available

### 20.3 G2O Export

The callback:

```cpp
write_current_estimates_callback()
```

allows exporting the current pose graph in G2O format for external analysis.

Associated topic:

```text
cslam/write_current_estimates
```

### 20.4 Visualization

Visualization support includes:

* Optional point cloud publishing.
* Voxel subsampling.
* Visualization point cloud messages.
* Pose graph visualization on:

```text
/cslam/viz/pose_graph
```

Visualization parameters:

```yaml
visualization:
  enable: false
  publishing_period_ms: 100
  voxel_size: 0.05
  max_range: 10.0
```

The system also provides a minimal visualization tool that opportunistically collects mapping data from robots in communication range. Querying mapping data for planning or visualization may require additional computation and communication.

### 20.5 Debug Topics

| Topic                             | Message Type         | Purpose                    |
| --------------------------------- | -------------------- | -------------------------- |
| `cslam/debug_optimization_result` | `OptimizationResult` | Debug optimized estimates. |
| `cslam/optimizer_state`           | `OptimizerState`     | Current optimizer state.   |
| `/cslam/viz/pose_graph`           | `PoseGraph`          | Visualization pose graph.  |

---

## 21. Error Handling and Robustness

## 21.1 Error Handling Patterns

| Scenario                            | Handling                                                                   |
| ----------------------------------- | -------------------------------------------------------------------------- |
| Optimization failure                | Catch exception and return initial estimates.                              |
| Timeout while waiting for neighbors | Reset to `IDLE` and log a warning / info message.                          |
| Graph not connected                 | Filter out non-connected robots and optimize only the connected component. |
| Empty estimates                     | Log warning and ignore the message.                                        |
| Log writing failure                 | Catch exception and log an error.                                          |
| Duplicate loop closures             | Deduplicate using `std::set`.                                              |
| Communication interruption          | Timeout prevents deadlock.                                                 |
| Partial topology                    | BFS connectivity allows operation on connected subsets.                    |

### 21.2 Robustness Mechanisms

* **GNC optimizer**

  * Reduces impact of outlier loop closures.

* **BFS connectivity check**

  * Supports partial multi-robot graph topologies.

* **Timeouts**

  * Prevent waiting forever for missing messages.

* **Asynchronous optimization**

  * Prevents optimization from blocking the main loop.

* **Neighbor manager bookkeeping**

  * Tracks exchanged measurements and avoids redundant bandwidth usage.

* **Sparse candidate prioritization**

  * Reduces unnecessary local descriptor / point cloud transmission.

* **Vertex cover-based feature exchange**

  * Avoids sending the same local features multiple times.

* **Covariance-based odometry rejection in LiDAR handler**

  * Rejects unreliable odometry updates.

* **No permanent central server**

  * A single server failure does not collapse the whole system.

* **Decentralized rendezvous**

  * Robots can optimize in temporary local groups.

### 21.3 Known Resource Constraints

The system is designed for mobile robots with limited:

* CPU resources
* Memory
* Network bandwidth
* Communication availability

Front-end communication load depends strongly on:

* Number of keyframes
* Descriptor size
* Whether local descriptors or point clouds are transmitted
* Visualization data queries

---

## 22. Design Patterns

| Pattern                                    | Usage                                                                                                              |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------ |
| **Strategy Pattern**                       | `ISensorHandler` interface with multiple implementations: `RGBDHandler`, `StereoHandler`, `LidarHandler`.          |
| **Template Method**                        | `MapManager<DataHandlerType>` supports different sensor handlers while reusing the same map-management pipeline.   |
| **State Machine**                          | `DecentralizedPGO` uses explicit optimizer states and transitions.                                                 |
| **Observer Pattern**                       | ROS 2 callbacks react asynchronously to odometry, loop closures, neighbors, pose graphs, and optimization results. |
| **Filter Chain**                           | `message_filters` synchronize multiple sensor streams.                                                             |
| **Producer-Consumer**                      | Queue-based sensor and keyframe processing in front-end handlers.                                                  |
| **Strategy Pattern for place recognition** | ScanContext, Co-SPlace, NetVLAD, and other descriptor approaches can be swapped.                                   |
| **Factory Pattern**                        | Sensor handler creation can be based on configuration.                                                             |
| **Broker / temporary coordinator**         | A broker is dynamically elected for front-end matching and local-feature exchange during rendezvous.               |

---

## 23. Extension Points

## 23.1 Back-End Extensions

Potential extension points:

1. **Optimizer selection**

   * Modify `is_optimizer()` to use criteria other than robot ID or origin ID.
   * Possible criteria:

     * available CPU
     * battery level
     * communication quality
     * graph size

2. **Noise model**

   * Customize `default_noise_model_` for different sensors or uncertainty estimates.

3. **Solver**

   * Replace or compare GNC with:

     * Dogleg
     * Levenberg-Marquardt
     * iSAM2
     * other GTSAM solvers

4. **GPS integration**

   * Extend partially implemented GPS support.

5. **Simulated rendezvous**

   * Extend the rendezvous simulation system.

6. **Anchor / reference-frame policy**

   * Modify how global reference frames are selected and propagated.

### 23.2 Front-End Extensions

Potential front-end extensions:

* Add support for new sensor types:

  * event cameras
  * RADAR
  * additional RGB-D cameras
  * additional LiDAR types
* Add new visual place recognition methods.
* Add new LiDAR place recognition methods.
* Improve geometric verification.
* Improve local descriptor compression.
* Add adaptive descriptor exchange policies.

### 23.3 Communication Extensions

Potential communication extensions:

* Adaptive communication based on network conditions.
* Improved MAC policies.
* Alternative broker election mechanisms.
* Bandwidth-aware local descriptor transmission.
* Sparse pose graph exchange policies.
* More robust ad-hoc networking behavior.

### 23.4 System-Level Extensions

Future extensions from the source documents include:

* Integration with navigation stacks.
* Improved optimizer selection strategies.
* Adaptive communication based on network conditions.
* Support for additional sensor types.
* Improved collaborative place recognition.
* Use as a testbed for:

  * place recognition
  * inter-robot loop closure detection
  * multi-robot pose graph optimization
  * collaborative perception research

---

## 24. Potential Inconsistencies / Items to Verify

This section preserves disagreements or ambiguous details from the source documents instead of silently resolving them.

### 24.1 GTSAM Optimizer Type

Different documents describe the optimizer differently:

* One document states the back-end uses a **GTSAM GNC optimizer**.
* One code snippet uses:

```cpp
gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
```

* One document describes the optimization state as:

```text
GTSAM iSAM2 optimization
```

* The supporting paper describes **Graduated Non-Convexity** with robust **Truncated Least Square** loss.
* One source phrase describes GNC as **Gain Scheduling**.
* Another source phrase describes GNC as **Generalized Concave**.
* Standard terminology for GNC in this context is usually **Graduated Non-Convexity**, but the implementation should be verified directly.

### 24.2 Optimizer Selection Rule

Two related but different descriptions appear:

1. Detailed back-end logic:

   * Lowest `origin_robot_id` wins.
   * If equal, lowest `robot_id` wins.
   * Robot must have odometry estimates.

2. High-level summary:

   * Robot with the lowest ID becomes the optimizer.

These may both be true at different abstraction levels, but the exact implemented rule should be verified in `DecentralizedPGO::is_optimizer()`.

### 24.3 Broker Selection vs Optimizer Selection vs Anchor Selection

The documents mention several elected roles:

| Role                       | Described Selection                                                                         |
| -------------------------- | ------------------------------------------------------------------------------------------- |
| Temporary front-end broker | Lowest robot ID in communication range.                                                     |
| Back-end optimizer         | Lowest `origin_robot_id`, tie-broken by `robot_id`; elsewhere described as lowest robot ID. |
| Anchor / reference frame   | First pose of robot with lowest ID or lowest reference-frame ID.                            |

These roles are related but not identical. The implementation should verify whether they are intentionally separate.

### 24.4 Repository URL

The source documents mention:

```markdown
[Swarm-SLAM](https://github.com/Swarm-SLAM)
```

and the supporting paper references:

```text
https://github.com/MISTLab/Swarm-SLAM
```

The canonical repository URL should be verified.

### 24.5 Message Package Naming

The documents use both:

```text
cslam_common_interfaces
```

and:

```text
src/cslam_interfaces/cslam_common_interfaces
```

They also mention:

```text
cslam_zenoh_interfaces
```

The exact ROS 2 package names and workspace layout should be verified against the repository.

### 24.6 Topic Namespacing

Some topics are relative:

```text
cslam/pose_graph
cslam/optimized_estimates
```

Others are absolute or robot-namespaced:

```text
/cslam/pose_graph
/r{ID}/cslam/optimized_estimates
/r{ID}/cslam/get_pose_graph
```

The exact namespace behavior should be verified in launch files and node initialization.

### 24.7 `cslam/write_current_estimates`

The callback:

```cpp
write_current_estimates_callback()
```

and topic:

```text
cslam/write_current_estimates
```

are documented in the callback table, but not in the main topic diagram. The exact message type should be verified.

### 24.8 LiDAR Front-End Description

The LiDAR front-end is described as:

* ICP-based registration.
* ScanContext descriptor generation.
* LiDAR place recognition.
* Covariance-based odometry rejection.
* Python implementation.

The exact division of responsibility between `lidar_handler_node.py`, `loop_closure_detection_node.py`, `scancontext.py`, `scancontext_matching.py`, and `icp_utils.py` should be verified.

### 24.9 `MapManager` Data Flow

One document states that `MapManager` receives keyframes from RTAB-Map. Another describes `RGBDHandler::send_keyframe()` as publishing to a Python node.

The precise flow between RTAB-Map, `RGBDHandler`, `MapManager`, and Python loop-closure nodes should be verified in the implementation.

### 24.10 Parameter Defaults

The detailed configuration table lists several defaults as `-`, meaning unspecified in the source documents.

Examples:

```text
robot_id
max_nb_robots
enable_logs
log_folder
enable_gps_recording
enable_simulated_rendezvous
```

Actual defaults should be verified in YAML files and parameter declarations.

### 24.11 `keyframe_generation_ratio_distance`

The LiDAR keyframe parameter is documented as:

```yaml
keyframe_generation_ratio_distance
```

The exact name should be verified because it is semantically distance-based but includes `ratio` in the name.

### 24.12 `gtsam::LabeledSymbol` Robot Label Range

The documented robot label scheme is:

```text
ROBOT_LABEL(id) = 'A' + id
```

If implemented literally, this may imply a practical label range tied to character labels. The actual supported `max_nb_robots` and label handling should be verified.

### 24.13 Temporary Centralization

The system is described as decentralized and without a central server. However:

* A temporary broker may be elected for matching.
* A temporary optimizer may aggregate graphs and compute optimization.

This is not necessarily inconsistent because the broker / optimizer is dynamic and local to communication range, but the distinction should be clearly documented in implementation-level docs.

---

## 25. References

* User-provided Markdown source document 1: **Swarm-SLAM Architecture Overview**.
* User-provided Markdown source document 2: **CSLAM Back-End Architecture Documentation**.
* User-provided Markdown source document 3: **CSLAM Project Summary**.
* Swarm-SLAM supporting paper PDF: **Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems**. 
* GTSAM Documentation: [https://gtsam.org/](https://gtsam.org/)
* ROS 2 Documentation: [https://docs.ros.org/](https://docs.ros.org/)
* Repository reference from source documents: [https://github.com/Swarm-SLAM](https://github.com/Swarm-SLAM)
* Repository reference from supporting paper: [https://github.com/MISTLab/Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM)
