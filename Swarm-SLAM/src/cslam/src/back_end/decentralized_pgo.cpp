#include "cslam/back_end/decentralized_pgo.h"

#include <algorithm>
#include <sstream>

#define MAP_FRAME_ID(id) "robot" + std::to_string(id) + "_map"
#define CURRENT_FRAME_ID(id) "robot" + std::to_string(id) + "_current_pose"
#define LATEST_OPTIMIZED_FRAME_ID(id) "robot" + std::to_string(id) + "_latest_optimized_pose"

using namespace cslam;

namespace {
const char *optimizer_state_to_string(OptimizerState state)
{
  switch (state)
  {
    case OptimizerState::IDLE:
      return "IDLE";
    case OptimizerState::WAITING_FOR_NEIGHBORS_INFO:
      return "WAITING_FOR_NEIGHBORS_INFO";
    case OptimizerState::POSEGRAPH_COLLECTION:
      return "POSEGRAPH_COLLECTION";
    case OptimizerState::WAITING_FOR_NEIGHBORS_POSEGRAPHS:
      return "WAITING_FOR_NEIGHBORS_POSEGRAPHS";
    case OptimizerState::START_OPTIMIZATION:
      return "START_OPTIMIZATION";
    case OptimizerState::OPTIMIZATION:
      return "OPTIMIZATION";
    default:
      return "UNKNOWN";
  }
}

std::string join_ids(const std::vector<unsigned int> &ids)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (i > 0)
    {
      oss << ", ";
    }
    oss << ids[i];
  }
  oss << "]";
  return oss.str();
}

std::string join_id_set(const std::set<unsigned int> &ids)
{
  std::ostringstream oss;
  oss << "[";
  size_t i = 0;
  for (auto id : ids)
  {
    if (i > 0)
    {
      oss << ", ";
    }
    oss << id;
    ++i;
  }
  oss << "]";
  return oss.str();
}

std::string join_connectivity_map(const std::map<unsigned int, bool> &is_connected)
{
  std::ostringstream oss;
  oss << "{";
  size_t i = 0;
  for (const auto &entry : is_connected)
  {
    if (i > 0)
    {
      oss << ", ";
    }
    oss << entry.first << ":" << (entry.second ? "true" : "false");
    ++i;
  }
  oss << "}";
  return oss.str();
}

std::set<unsigned int> robot_ids_in_values(const gtsam::Values &values)
{
  std::set<unsigned int> robot_ids;
  for (const auto key : values.keys())
  {
    robot_ids.insert(ROBOT_ID(gtsam::LabeledSymbol(key).label()));
  }
  return robot_ids;
}

bool has_pose_values_from_other_robots(const gtsam::Values &values,
                                       unsigned int local_robot_id)
{
  for (const auto key : values.keys())
  {
    if (ROBOT_ID(gtsam::LabeledSymbol(key).label()) != local_robot_id)
    {
      return true;
    }
  }
  return false;
}

size_t insert_missing_pose_values(gtsam::Values::shared_ptr target,
                                  const gtsam::Values &source)
{
  size_t inserted = 0;
  for (const auto key : source.keys())
  {
    if (!target->exists(key))
    {
      target->insert(key, source.at<gtsam::Pose3>(key));
      inserted++;
    }
  }
  return inserted;
}

gtsam::Values::shared_ptr pose_values_for_robot(const gtsam::Values &source,
                                                unsigned int robot_id)
{
  auto values = boost::make_shared<gtsam::Values>();
  for (const auto key : source.keys())
  {
    if (ROBOT_ID(gtsam::LabeledSymbol(key).label()) == robot_id)
    {
      values->insert(key, source.at<gtsam::Pose3>(key));
    }
  }
  return values;
}

gtsam::Values::shared_ptr local_odometry_with_relayed_remote_values(
    const gtsam::Values::shared_ptr &local_odometry,
    const gtsam::Values::shared_ptr &current_estimates,
    unsigned int local_robot_id)
{
  auto values = boost::make_shared<gtsam::Values>();
  values->insert(*local_odometry);
  for (const auto key : current_estimates->keys())
  {
    if (ROBOT_ID(gtsam::LabeledSymbol(key).label()) == local_robot_id ||
        values->exists(key))
    {
      continue;
    }
    values->insert(key, current_estimates->at<gtsam::Pose3>(key));
  }
  return values;
}

std::pair<gtsam::Key, gtsam::Key> normalized_factor_keys(gtsam::Key key0,
                                                         gtsam::Key key1)
{
  if (key0 < key1)
  {
    return {key0, key1};
  }
  return {key1, key0};
}

bool append_between_factor_if_unique(
    const gtsam::BetweenFactor<gtsam::Pose3> &factor,
    const gtsam::NonlinearFactorGraph::shared_ptr &graph,
    std::set<std::pair<gtsam::Key, gtsam::Key>> &added_factors,
    const gtsam::Values *required_values = nullptr)
{
  if (required_values != nullptr &&
      (!required_values->exists(factor.key1()) ||
       !required_values->exists(factor.key2())))
  {
    return false;
  }

  const auto keys = normalized_factor_keys(factor.key1(), factor.key2());
  if (added_factors.count(keys) > 0)
  {
    return false;
  }

  graph->push_back(factor);
  added_factors.insert(keys);
  return true;
}

bool append_factor_if_unique(
    const gtsam::NonlinearFactor::shared_ptr &factor,
    const gtsam::NonlinearFactorGraph::shared_ptr &graph,
    std::set<std::pair<gtsam::Key, gtsam::Key>> &added_factors,
    const gtsam::Values *required_values = nullptr)
{
  auto between_factor =
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
  if (!between_factor)
  {
    return false;
  }
  return append_between_factor_if_unique(
      *between_factor, graph, added_factors, required_values);
}

size_t append_factor_graph_if_unique(
    const gtsam::NonlinearFactorGraph &source,
    const gtsam::NonlinearFactorGraph::shared_ptr &target,
    std::set<std::pair<gtsam::Key, gtsam::Key>> &added_factors,
    const gtsam::Values *required_values = nullptr)
{
  size_t inserted = 0;
  for (const auto &factor : source)
  {
    if (append_factor_if_unique(
            factor, target, added_factors, required_values))
    {
      inserted++;
    }
  }
  return inserted;
}

size_t append_factor_graph_if_unique(
    const gtsam::NonlinearFactorGraph::shared_ptr &source,
    const gtsam::NonlinearFactorGraph::shared_ptr &target,
    std::set<std::pair<gtsam::Key, gtsam::Key>> &added_factors,
    const gtsam::Values *required_values = nullptr)
{
  return append_factor_graph_if_unique(
      *source, target, added_factors, required_values);
}

size_t append_reconstructed_odometry_edges(
    const gtsam::Values &values,
    unsigned int local_robot_id,
    const gtsam::SharedNoiseModel &noise_model,
    const gtsam::NonlinearFactorGraph::shared_ptr &graph,
    std::set<std::pair<gtsam::Key, gtsam::Key>> &added_factors)
{
  std::map<unsigned int, std::vector<gtsam::Key>> keys_by_robot;
  size_t inserted = 0;
  for (const auto key : values.keys())
  {
    const auto symbol = gtsam::LabeledSymbol(key);
    const unsigned int robot_id = ROBOT_ID(symbol.label());
    if (robot_id != local_robot_id)
    {
      keys_by_robot[robot_id].push_back(key);
    }
  }

  for (auto &entry : keys_by_robot)
  {
    auto &keys = entry.second;
    std::sort(keys.begin(), keys.end(), [](gtsam::Key lhs, gtsam::Key rhs) {
      return gtsam::LabeledSymbol(lhs).index() <
             gtsam::LabeledSymbol(rhs).index();
    });

    for (size_t i = 1; i < keys.size(); i++)
    {
      const auto previous_key = keys[i - 1];
      const auto current_key = keys[i];
      const auto previous_pose = values.at<gtsam::Pose3>(previous_key);
      const auto current_pose = values.at<gtsam::Pose3>(current_key);
      gtsam::BetweenFactor<gtsam::Pose3> factor(
          previous_key, current_key, previous_pose.between(current_pose),
          noise_model);
      if (append_between_factor_if_unique(factor, graph, added_factors))
      {
        inserted++;
      }
    }
  }
  return inserted;
}
} // namespace

DecentralizedPGO::DecentralizedPGO(std::shared_ptr<rclcpp::Node> &node)
    : node_(node), max_waiting_time_sec_(60, 0)
{
  node_->get_parameter("max_nb_robots", max_nb_robots_);
  node_->get_parameter("robot_id", robot_id_);
  node_->get_parameter("backend.pose_graph_optimization_start_period_ms",
                       pose_graph_optimization_start_period_ms_);
  node_->get_parameter("backend.pose_graph_optimization_loop_period_ms",
                       pose_graph_optimization_loop_period_ms_);
  node_->get_parameter("backend.enable_broadcast_tf_frames",
                       enable_broadcast_tf_frames_);
  node_->get_parameter("neighbor_management.heartbeat_period_sec", heartbeat_period_sec_);
  node->get_parameter("evaluation.enable_logs",
                      enable_logs_);
  node->get_parameter("evaluation.log_folder",
                      log_folder_);
  node->get_parameter("evaluation.enable_gps_recording",
                      enable_gps_recording_);
  node->get_parameter("evaluation.enable_simulated_rendezvous", enable_simulated_rendezvous_);
  std::string rendezvous_schedule_file;
  node->get_parameter("evaluation.rendezvous_schedule_file", rendezvous_schedule_file);
  node->get_parameter("evaluation.enable_pose_timestamps_recording", enable_pose_timestamps_recording_);
  node_->get_parameter("visualization.enable",
                       enable_visualization_);
  node_->get_parameter("visualization.publishing_period_ms",
                       visualization_period_ms_);

  int max_waiting_param;
  node_->get_parameter("backend.max_waiting_time_sec", max_waiting_param);
  max_waiting_time_sec_ = rclcpp::Duration(max_waiting_param, 0);

  anchor_symbol_ = gtsam::LabeledSymbol();
  warned_missing_initial_keyframe_ = false;

  odometry_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::KeyframeOdom>(
          "cslam/keyframe_odom",
          rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
          std::bind(&DecentralizedPGO::odometry_callback, this,
                    std::placeholders::_1));

  intra_robot_loop_closure_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::IntraRobotLoopClosure>(
      "cslam/intra_robot_loop_closure", 1000,
      std::bind(&DecentralizedPGO::intra_robot_loop_closure_callback, this,
                std::placeholders::_1));

  inter_robot_loop_closure_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::InterRobotLoopClosure>(
      "/cslam/inter_robot_loop_closure", 1000,
      std::bind(&DecentralizedPGO::inter_robot_loop_closure_callback, this,
                std::placeholders::_1));

  write_current_estimates_subscriber_ =
      node->create_subscription<std_msgs::msg::String>(
          "cslam/print_current_estimates", 100,
          std::bind(&DecentralizedPGO::write_current_estimates_callback, this,
                    std::placeholders::_1));

  rotation_default_noise_std_ = 0.01;
  translation_default_noise_std_ = 0.1;
  Eigen::VectorXd sigmas(6);
  sigmas << rotation_default_noise_std_, rotation_default_noise_std_,
      rotation_default_noise_std_, translation_default_noise_std_,
      translation_default_noise_std_, translation_default_noise_std_;
  default_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  Eigen::VectorXd loop_sigmas(6);
  loop_sigmas << 0.05, 0.05, 0.05, 0.50, 0.50, 0.50;
  auto loop_base_noise = gtsam::noiseModel::Diagonal::Sigmas(loop_sigmas);
  loop_closure_noise_model_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.0), loop_base_noise);
  pose_graph_ = boost::make_shared<gtsam::NonlinearFactorGraph>();
  current_pose_estimates_ = boost::make_shared<gtsam::Values>();
  odometry_pose_estimates_ = boost::make_shared<gtsam::Values>();
  // Consecutive revisit keyframes can produce many nearly identical intra-loop
  // closures. Even when each individual match is good, counting all of them as
  // independent constraints can over-bias the optimizer and bend the graph.
  intra_loop_redundancy_window_source_ = 4;
  intra_loop_redundancy_window_target_ = 4;

  // Optimization timers
  optimization_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_start_period_ms_),
      std::bind(&DecentralizedPGO::optimization_callback, this));

  optimization_loop_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_),
      std::bind(&DecentralizedPGO::optimization_loop_callback, this));

  if (enable_visualization_)
  {
    RCLCPP_INFO(node_->get_logger(), "Visualization enabled.");
    visualization_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(visualization_period_ms_),
        std::bind(&DecentralizedPGO::visualization_callback, this));
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Visualization disabled.");
  }

  // Publishers for optimization result
  debug_optimization_result_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizationResult>(
          "cslam/debug_optimization_result", 100);

  for (unsigned int i = 0; i < max_nb_robots_; i++)
  {
    optimized_estimates_publishers_.insert(
        {i, node->create_publisher<
                cslam_common_interfaces::msg::OptimizationResult>(
                "/r" + std::to_string(i) + "/cslam/optimized_estimates", 100)});
  }

  optimized_estimates_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::OptimizationResult>(
      "cslam/optimized_estimates", 100,
      std::bind(&DecentralizedPGO::optimized_estimates_callback, this,
                std::placeholders::_1));

  optimized_pose_estimate_publisher_ = node->create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/r" + std::to_string(robot_id_) + "/cslam/current_pose_estimate", 100);

  optimizer_state_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::OptimizerState>(
          "cslam/optimizer_state", 100);

  // Initialize inter-robot loop closures measurements
  for (unsigned int i = 0; i < max_nb_robots_; i++)
  {
    for (unsigned int j = i + 1; j < max_nb_robots_; j++)
    {
      inter_robot_loop_closures_.insert(
          {{i, j}, std::vector<gtsam::BetweenFactor<gtsam::Pose3>>()});
    }
  }

  // Get neighbors ROS 2 objects
  get_current_neighbors_publisher_ =
      node->create_publisher<std_msgs::msg::String>("cslam/get_current_neighbors",
                                                    100);

  current_neighbors_subscriber_ = node->create_subscription<
      cslam_common_interfaces::msg::RobotIdsAndOrigin>(
      "cslam/current_neighbors", 100,
      std::bind(&DecentralizedPGO::current_neighbors_callback, this,
                std::placeholders::_1));

  // PoseGraph ROS 2 objects
  for (unsigned int i = 0; i < max_nb_robots_; i++)
  {
    get_pose_graph_publishers_.insert(
        {i, node->create_publisher<cslam_common_interfaces::msg::RobotIds>(
                "/r" + std::to_string(i) + "/cslam/get_pose_graph", 100)});
    received_pose_graphs_.insert({i, false});
  }

  get_pose_graph_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::RobotIds>(
          "cslam/get_pose_graph", 100,
          std::bind(&DecentralizedPGO::get_pose_graph_callback, this,
                    std::placeholders::_1));

  pose_graph_publisher_ =
      node->create_publisher<cslam_common_interfaces::msg::PoseGraph>(
          "/cslam/pose_graph", 100);

  pose_graph_subscriber_ =
      node->create_subscription<cslam_common_interfaces::msg::PoseGraph>(
          "/cslam/pose_graph", 100,
          std::bind(&DecentralizedPGO::pose_graph_callback, this,
                    std::placeholders::_1));

  visualization_pose_graph_publisher_ =
      node->create_publisher<cslam_common_interfaces::msg::PoseGraph>(
          "/cslam/viz/pose_graph", 100);

  merged_visualization_pose_graph_publisher_ =
      node->create_publisher<cslam_common_interfaces::msg::PoseGraph>(
          "cslam/viz/merged_pose_graph", 100);

  // Optimizer
  optimizer_state_ = OptimizerState::IDLE;
  is_waiting_ = false;
  optimization_count_ = 0;

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  if (enable_broadcast_tf_frames_)
  {
    tf_broadcaster_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(pose_graph_optimization_loop_period_ms_),
        std::bind(&DecentralizedPGO::broadcast_tf_callback, this));
  }

  heartbeat_publisher_ =
      node_->create_publisher<std_msgs::msg::UInt32>("cslam/heartbeat", 10);
  heartbeat_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds((unsigned int)heartbeat_period_sec_ * 1000),
      std::bind(&DecentralizedPGO::heartbeat_timer_callback, this));

  reference_frame_per_robot_publisher_ =
      node_->create_publisher<cslam_common_interfaces::msg::ReferenceFrames>(
          "cslam/reference_frames", rclcpp::QoS(1).transient_local());

  origin_robot_id_ = robot_id_;

  if (enable_logs_)
  {
    logger_ = std::make_shared<Logger>(node_, robot_id_, max_nb_robots_, log_folder_);
  }

  if (enable_simulated_rendezvous_)
  {
    sim_rdv_ = std::make_shared<SimulatedRendezVous>(node_, rendezvous_schedule_file, robot_id_);
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[DEBUG_BACKEND_PIPELINE] pose_graph_manager ready ns=%s robot_id=%u max_nb_robots=%u "
      "keyframe_odom_sub=cslam/keyframe_odom optimized_pose_pub=/r%u/cslam/current_pose_estimate "
      "broadcast_tf=%s opt_start_period_ms=%u opt_loop_period_ms=%u",
      node_->get_namespace(), robot_id_, max_nb_robots_, robot_id_,
      enable_broadcast_tf_frames_ ? "true" : "false",
      pose_graph_optimization_start_period_ms_, pose_graph_optimization_loop_period_ms_);

  RCLCPP_INFO(node_->get_logger(), "Initialization done.");
}

void DecentralizedPGO::reinitialize_received_pose_graphs()
{
  for (unsigned int i = 0; i < max_nb_robots_; i++)
  {
    received_pose_graphs_[i] = false;
  }
  other_robots_graph_and_estimates_.clear();
  received_pose_graphs_connectivity_.clear();
}

bool DecentralizedPGO::check_received_pose_graphs()
{
  bool received_all = true;
  for (auto id : current_neighbors_ids_.robots.ids)
  {
    received_all &= received_pose_graphs_[id];
  }
  return received_all;
}

void DecentralizedPGO::odometry_callback(
    const cslam_common_interfaces::msg::KeyframeOdom::ConstSharedPtr msg)
{
  static uint64_t odom_cb_count = 0;
  odom_cb_count++;

  gtsam::Pose3 current_estimate = odometry_msg_to_pose3(msg->odom);
  gtsam::LabeledSymbol symbol(GRAPH_LABEL, ROBOT_LABEL(robot_id_), msg->id);

  if (odom_cb_count <= 5 || odom_cb_count % 50 == 0)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] keyframe_odom received count=%lu id=%u stamp=%u.%u frame=%s child=%s "
        "odom_values=%lu current_values=%lu",
        odom_cb_count, msg->id,
        msg->odom.header.stamp.sec, msg->odom.header.stamp.nanosec,
        msg->odom.header.frame_id.c_str(), msg->odom.child_frame_id.c_str(),
        odometry_pose_estimates_->size(), current_pose_estimates_->size());
  }

  odometry_pose_estimates_->insert(symbol, current_estimate);
  if (anchor_symbol_ == gtsam::LabeledSymbol())
  {
    anchor_symbol_ = symbol;
    current_pose_estimates_->insert(symbol, current_estimate);
    if (msg->id != 0 && !warned_missing_initial_keyframe_)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "[DEBUG_BACKEND_PIPELINE] Missing keyframe id=0, initializing graph from first received keyframe id=%u.",
          msg->id);
      warned_missing_initial_keyframe_ = true;
    }
  }
  else if (msg->id == 0)
  {
    current_pose_estimates_->insert(symbol, current_estimate);
  }

  if (latest_local_symbol_ != gtsam::LabeledSymbol())
  {
    gtsam::Pose3 odom_diff = latest_local_pose_.inverse() * current_estimate;
    gtsam::BetweenFactor<gtsam::Pose3> factor(latest_local_symbol_, symbol,
                                              odom_diff, default_noise_model_);
    pose_graph_->push_back(factor);
  }

  if (enable_gps_recording_)
  {
    gps_data_.insert({msg->id, msg->gps});
  }

  // Update latest pose
  latest_local_pose_ = current_estimate;
  latest_local_symbol_ = symbol;
  latest_local_pose_stamp_ = msg->odom.header.stamp;

  if (enable_pose_timestamps_recording_)
  {
    logger_->log_pose_timestamp(symbol, msg->odom.header.stamp.sec, msg->odom.header.stamp.nanosec);
  }
}

void DecentralizedPGO::intra_robot_loop_closure_callback(
    const cslam_common_interfaces::msg::IntraRobotLoopClosure::
        ConstSharedPtr msg)
{
  if (msg->success)
  {
    for (const auto &accepted_pair : accepted_intra_robot_loop_pairs_)
    {
      const int source_delta =
          std::abs(static_cast<int>(msg->keyframe0_id) - static_cast<int>(accepted_pair.first));
      const int target_delta =
          std::abs(static_cast<int>(msg->keyframe1_id) - static_cast<int>(accepted_pair.second));

      if (source_delta <= intra_loop_redundancy_window_source_ &&
          target_delta <= intra_loop_redundancy_window_target_)
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "[DEBUG_BACKEND_PIPELINE] Skipping redundant intra-robot loop closure (%u, %u) near accepted (%u, %u).",
            msg->keyframe0_id, msg->keyframe1_id, accepted_pair.first,
            accepted_pair.second);
        return;
      }
    }

    // Registration returns the transform from the matched frame back to the
    // query frame. GTSAM BetweenFactor(from, to) expects from.between(to).
    gtsam::Pose3 measurement = transform_msg_to_pose3(msg->transform).inverse();

    gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, ROBOT_LABEL(robot_id_),
                                     msg->keyframe0_id);
    gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, ROBOT_LABEL(robot_id_),
                                   msg->keyframe1_id);

    gtsam::BetweenFactor<gtsam::Pose3> factor =
        gtsam::BetweenFactor<gtsam::Pose3>(symbol_from, symbol_to, measurement,
                                           loop_closure_noise_model_);

    pose_graph_->push_back(factor);
    accepted_intra_robot_loop_pairs_.emplace_back(msg->keyframe0_id,
                                                  msg->keyframe1_id);
    RCLCPP_INFO(node_->get_logger(), "New intra-robot loop closure (%d, %d).", msg->keyframe0_id, msg->keyframe1_id);
  }
}

void DecentralizedPGO::inter_robot_loop_closure_callback(
    const cslam_common_interfaces::msg::InterRobotLoopClosure::
        ConstSharedPtr msg)
{
  if (msg->success)
  {
    // Registration returns the transform from the matched frame back to the
    // query frame. GTSAM BetweenFactor(from, to) expects from.between(to).
    gtsam::Pose3 measurement = transform_msg_to_pose3(msg->transform).inverse();

    unsigned char robot0_c = ROBOT_LABEL(msg->robot0_id);
    gtsam::LabeledSymbol symbol_from(GRAPH_LABEL, robot0_c,
                                     msg->robot0_keyframe_id);
    unsigned char robot1_c = ROBOT_LABEL(msg->robot1_id);
    gtsam::LabeledSymbol symbol_to(GRAPH_LABEL, robot1_c, msg->robot1_keyframe_id);

    gtsam::BetweenFactor<gtsam::Pose3> factor =
        gtsam::BetweenFactor<gtsam::Pose3>(symbol_from, symbol_to, measurement,
                                           loop_closure_noise_model_);

    inter_robot_loop_closures_[{std::min(msg->robot0_id, msg->robot1_id),
                                std::max(msg->robot0_id, msg->robot1_id)}]
        .push_back(factor);
    if (msg->robot0_id == robot_id_)
    {
      connected_robots_.insert(msg->robot1_id);
    }
    else if (msg->robot1_id == robot_id_)
    {
      connected_robots_.insert(msg->robot0_id);
    }

    const auto pair_key =
        std::make_pair(std::min(msg->robot0_id, msg->robot1_id),
                       std::max(msg->robot0_id, msg->robot1_id));
    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] inter_robot_loop_closure accepted pair=(%u,%u) "
        "kf0=%u kf1=%u pair_count=%zu local_connected=%s",
        msg->robot0_id, msg->robot1_id, msg->robot0_keyframe_id,
        msg->robot1_keyframe_id,
        inter_robot_loop_closures_[pair_key].size(),
        join_id_set(connected_robots_).c_str());
  }
}

void DecentralizedPGO::write_current_estimates_callback(
    const std_msgs::msg::String::ConstSharedPtr msg)
{
  try{
    gtsam::writeG2o(*pose_graph_, *current_pose_estimates_, msg->data);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error while writing estimates: %s", e.what());
  }
}

void DecentralizedPGO::current_neighbors_callback(
    const cslam_common_interfaces::msg::RobotIdsAndOrigin::ConstSharedPtr msg)
{
  current_neighbors_ids_ = *msg;
  end_waiting();
  const bool local_is_optimizer = is_optimizer();
  if (local_is_optimizer)
  {
    optimizer_state_ = OptimizerState::POSEGRAPH_COLLECTION;
  }
  else
  {
    optimizer_state_ = OptimizerState::IDLE;
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[DEBUG_BACKEND_PIPELINE] neighbors update robots=%s origins=%s is_optimizer=%s odom_values=%zu",
      join_ids(current_neighbors_ids_.robots.ids).c_str(),
      join_ids(current_neighbors_ids_.origins.ids).c_str(),
      local_is_optimizer ? "true" : "false",
      odometry_pose_estimates_->size());
}

bool DecentralizedPGO::is_optimizer()
{
  // Here we could implement a different priority check
  bool is_optimizer = true;
  for (unsigned int i = 0; i < current_neighbors_ids_.origins.ids.size(); i++)
  {
    if (origin_robot_id_ > current_neighbors_ids_.origins.ids[i])
    {
      is_optimizer = false;
    }
    else if (origin_robot_id_ == current_neighbors_ids_.origins.ids[i] &&
             robot_id_ > current_neighbors_ids_.robots.ids[i])
    {
      is_optimizer = false;
    }
  }
  if (odometry_pose_estimates_->size() == 0)
  {
    is_optimizer = false;
  }
  return is_optimizer;
}

cslam_common_interfaces::msg::PoseGraph DecentralizedPGO::fill_pose_graph_msg(){
  auto current_robots_ids = current_neighbors_ids_;
  current_robots_ids.robots.ids.push_back(robot_id_);
  return fill_pose_graph_msg(current_robots_ids.robots);
}

cslam_common_interfaces::msg::PoseGraph DecentralizedPGO::fill_pose_graph_msg(const cslam_common_interfaces::msg::RobotIds& msg){
  (void)msg;
  cslam_common_interfaces::msg::PoseGraph out_msg;
  out_msg.robot_id = robot_id_;
  out_msg.origin_robot_id = origin_robot_id_;

  const bool has_merged_estimates = has_pose_values_from_other_robots(
      *current_pose_estimates_, robot_id_);
  auto values_to_send =
      has_merged_estimates
          ? local_odometry_with_relayed_remote_values(
                odometry_pose_estimates_, current_pose_estimates_, robot_id_)
          : odometry_pose_estimates_;
  out_msg.values = gtsam_values_to_msg(values_to_send);

  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  std::set<std::pair<gtsam::Key, gtsam::Key>> added_factors;
  append_factor_graph_if_unique(pose_graph_, graph, added_factors);
  if (has_merged_estimates)
  {
    append_reconstructed_odometry_edges(
        *values_to_send, robot_id_, default_noise_model_, graph,
        added_factors);
  }

  std::set<unsigned int> connected_robots(connected_robots_.begin(),
                                          connected_robots_.end());
  std::set<unsigned int> value_robot_ids = robot_ids_in_values(*values_to_send);

  for (auto robot0_id : value_robot_ids)
  {
    for (auto robot1_id : value_robot_ids)
    {
      if (robot0_id >= robot1_id)
      {
        continue;
      }

      unsigned int min_robot_id = std::min(robot0_id, robot1_id);
      unsigned int max_robot_id = std::max(robot0_id, robot1_id);
      for (const auto &factor :
           inter_robot_loop_closures_[{min_robot_id, max_robot_id}])
      {
        if (values_to_send->exists(factor.key1()) &&
            values_to_send->exists(factor.key2()) &&
            append_between_factor_if_unique(
                factor, graph, added_factors, values_to_send.get()))
        {
          connected_robots.insert(min_robot_id);
          connected_robots.insert(max_robot_id);
        }
      }
    }
  }

  out_msg.edges = gtsam_factors_to_msg(graph);
  for (auto id : connected_robots)
  {
    if (id != robot_id_)
    {
      out_msg.connected_robots.ids.push_back(id);
    }
  }

  if (enable_gps_recording_) {
    for (auto gps : gps_data_) {
      out_msg.gps_values_idx.push_back(gps.first);
      out_msg.gps_values.push_back(gps.second);
    }
  }
  
  // If logging, add extra data
  if (enable_logs_) {
    logger_->fill_msg(out_msg);
  }

  return out_msg;
}

void DecentralizedPGO::get_pose_graph_callback(
    const cslam_common_interfaces::msg::RobotIds::ConstSharedPtr msg)
{
  if (enable_simulated_rendezvous_ && !sim_rdv_->is_alive())
  {
    return;
  }
  auto out_msg = fill_pose_graph_msg(*msg);
  pose_graph_publisher_->publish(out_msg);
  tentative_local_pose_at_latest_optimization_ = latest_local_pose_;
  tentative_local_symbol_at_latest_optimization_ = latest_local_symbol_;
}

void DecentralizedPGO::pose_graph_callback(
    const cslam_common_interfaces::msg::PoseGraph::ConstSharedPtr msg)
{
  if (enable_simulated_rendezvous_ && !sim_rdv_->is_alive())
  {
    return;
  }
  if (optimizer_state_ == OptimizerState::WAITING_FOR_NEIGHBORS_POSEGRAPHS)
  {
    other_robots_graph_and_estimates_[msg->robot_id] =
        {edges_msg_to_gtsam(msg->edges), values_msg_to_gtsam(msg->values)};
    received_pose_graphs_[msg->robot_id] = true;
    received_pose_graphs_connectivity_[msg->robot_id] =
        msg->connected_robots.ids;

    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] pose_graph received from=%u values=%zu edges=%zu connected_robots=%s all_received=%s",
        msg->robot_id, msg->values.size(), msg->edges.size(),
        join_ids(msg->connected_robots.ids).c_str(),
        check_received_pose_graphs() ? "true" : "false");
      
    if (enable_logs_){
      logger_->add_pose_graph_log_info(*msg);
    }
    if (check_received_pose_graphs())
    {
      end_waiting();
      optimizer_state_ = OptimizerState::START_OPTIMIZATION;
      if (enable_logs_){
        logger_->add_pose_graph_log_info(fill_pose_graph_msg());
      }
    }
  }
}

std::map<unsigned int, bool> DecentralizedPGO::connected_robot_pose_graph()
{
  std::map<unsigned int, std::set<unsigned int>> connectivity_graph;
  auto add_connectivity_edge = [&connectivity_graph](unsigned int a,
                                                     unsigned int b) {
    connectivity_graph[a].insert(b);
    connectivity_graph[b].insert(a);
  };

  for (auto id : connected_robots_)
  {
    add_connectivity_edge(robot_id_, id);
  }

  for (const auto &entry : received_pose_graphs_connectivity_)
  {
    for (auto id : entry.second)
    {
      add_connectivity_edge(entry.first, id);
    }
  }

  std::map<unsigned int, bool> is_robot_connected;
  is_robot_connected.insert({robot_id_, true});
  for (auto id : current_neighbors_ids_.robots.ids)
  {
    is_robot_connected.insert({id, false});
  }

  for (const auto &entry : connectivity_graph)
  {
    is_robot_connected.insert({entry.first, false});
    for (auto id : entry.second)
    {
      is_robot_connected.insert({id, false});
    }
  }

  // Breadth First Search
  std::map<unsigned int, bool> visited;
  std::list<unsigned int> queue;

  unsigned int current_id = robot_id_;
  visited[current_id] = true;
  queue.push_back(current_id);

  while (!queue.empty())
  {
    current_id = queue.front();
    queue.pop_front();

    for (auto id : connectivity_graph[current_id])
    {
      is_robot_connected[id] = true;

      if (visited.count(id) == 0 || !visited[id])
      {
        visited[id] = true;
        queue.push_back(id);
      }
    }
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[DEBUG_BACKEND_PIPELINE] pose_graph connectivity local_connected=%s result=%s",
      join_id_set(connected_robots_).c_str(),
      join_connectivity_map(is_robot_connected).c_str());

  return is_robot_connected;
}

void DecentralizedPGO::resquest_current_neighbors()
{
  if (enable_simulated_rendezvous_ && !sim_rdv_->is_alive())
  {
    return;
  }
  get_current_neighbors_publisher_->publish(std_msgs::msg::String());
}

void DecentralizedPGO::start_waiting()
{
  if (optimizer_state_ == OptimizerState::IDLE)
  {
    optimizer_state_ = OptimizerState::WAITING_FOR_NEIGHBORS_INFO;
  }
  else if (optimizer_state_ == OptimizerState::POSEGRAPH_COLLECTION)
  {
    optimizer_state_ = OptimizerState::WAITING_FOR_NEIGHBORS_POSEGRAPHS;
  }
  is_waiting_ = true;
  start_waiting_time_ = node_->now();
}

void DecentralizedPGO::end_waiting() { is_waiting_ = false; }

bool DecentralizedPGO::is_waiting() { return is_waiting_; }

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

void DecentralizedPGO::optimization_callback()
{
  if (enable_simulated_rendezvous_ && !sim_rdv_->is_alive())
  {
    return;
  }
  if (optimizer_state_ == OptimizerState::IDLE &&
      odometry_pose_estimates_->size() > 0)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] optimization cycle trigger odom_values=%lu",
        odometry_pose_estimates_->size());
    reinitialize_received_pose_graphs();
    resquest_current_neighbors();
    start_waiting();
  }
}

std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
DecentralizedPGO::aggregate_pose_graphs()
{
  // Check connectivity
  auto is_pose_graph_connected = connected_robot_pose_graph();
  // Aggregate graphs
  auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
  auto estimates = boost::make_shared<gtsam::Values>();
  std::set<std::pair<gtsam::Key, gtsam::Key>> added_factors;
  // Local graph
  const bool has_merged_estimates = has_pose_values_from_other_robots(
      *current_pose_estimates_, robot_id_);
  auto local_values =
      has_merged_estimates
          ? local_odometry_with_relayed_remote_values(
                odometry_pose_estimates_, current_pose_estimates_, robot_id_)
          : odometry_pose_estimates_;
  estimates->insert(*local_values);
  const size_t inserted_local_factors =
      append_factor_graph_if_unique(pose_graph_, graph, added_factors);
  size_t inserted_relayed_factors = 0;
  tentative_local_pose_at_latest_optimization_ = latest_local_pose_;
  tentative_local_symbol_at_latest_optimization_ = latest_local_symbol_;
  size_t inserted_remote_values = 0;
  size_t inserted_inter_robot_loop_closures = 0;
  size_t inserted_remote_factors = 0;

  // Add other robots graphs
  for (auto id : current_neighbors_ids_.robots.ids)
  {
    if (is_pose_graph_connected[id])
    {
      inserted_remote_values += insert_missing_pose_values(
          estimates, *other_robots_graph_and_estimates_[id].second);
    }
  }

  // Add local inter-robot loop closures
  const auto included_robot_ids = robot_ids_in_values(*estimates);
  for (auto robot0_id : included_robot_ids)
  {
    for (auto robot1_id : included_robot_ids)
    {
      if (robot0_id >= robot1_id)
      {
        continue;
      }

      if (is_pose_graph_connected[robot0_id] &&
          is_pose_graph_connected[robot1_id])
      {
        unsigned int min_id = std::min(robot0_id, robot1_id);
        unsigned int max_id = std::max(robot0_id, robot1_id);
        for (const auto &factor : inter_robot_loop_closures_[{min_id, max_id}])
        {
          if (append_between_factor_if_unique(
                  factor, graph, added_factors, estimates.get()))
          {
            inserted_inter_robot_loop_closures++;
          }
        }
      }
    }
  }
  // Add other robots factors
  for (auto id : current_neighbors_ids_.robots.ids)
  {

    for (const auto &factor_ : *other_robots_graph_and_estimates_[id].first)
    {
      auto factor =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
              factor_);
      if (!factor)
      {
        continue;
      }
      unsigned int robot0_id =
          ROBOT_ID(gtsam::LabeledSymbol(factor->key1()).label());
      unsigned int robot1_id =
          ROBOT_ID(gtsam::LabeledSymbol(factor->key2()).label());
      if (is_pose_graph_connected[robot0_id] &&
          is_pose_graph_connected[robot1_id])
      {
        if (append_factor_if_unique(
                factor_, graph, added_factors, estimates.get()))
        {
          inserted_remote_factors++;
        }
      }
    }
  }
  if (has_merged_estimates)
  {
    inserted_relayed_factors += append_reconstructed_odometry_edges(
        *local_values, robot_id_, default_noise_model_, graph,
        added_factors);
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[DEBUG_BACKEND_PIPELINE] aggregate_pose_graphs connected=%s local_values=%zu remote_values=%zu "
      "total_estimates=%zu local_factors=%zu remote_factors=%zu inter_robot_factors=%zu total_factors=%zu",
      join_connectivity_map(is_pose_graph_connected).c_str(),
      odometry_pose_estimates_->size(),
      inserted_remote_values,
      estimates->size(),
      inserted_local_factors + inserted_relayed_factors,
      inserted_remote_factors,
      inserted_inter_robot_loop_closures,
      graph->size());
  return {graph, estimates};
}

void DecentralizedPGO::optimized_estimates_callback(
    const cslam_common_interfaces::msg::OptimizationResult::ConstSharedPtr
        msg)
{
  if (enable_simulated_rendezvous_ && !sim_rdv_->is_alive())
  {
    return;
  }
  if (msg->estimates.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "[DEBUG_BACKEND_PIPELINE] optimized_estimates received empty estimates.");
  }

  if (odometry_pose_estimates_->size() > 0 && msg->estimates.size() > 0)
  {
    current_pose_estimates_ = values_msg_to_gtsam(msg->estimates);
    origin_robot_id_ = msg->origin_robot_id;

    gtsam::Pose3 first_pose;
    if (anchor_symbol_ != gtsam::LabeledSymbol() &&
        current_pose_estimates_->exists(anchor_symbol_))
    {
      first_pose = current_pose_estimates_->at<gtsam::Pose3>(anchor_symbol_);
    }
    update_transform_to_origin(first_pose);

    if (enable_logs_) {
      try{
        logger_->write_logs();
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Writing logs failed: %s", e.what());
      }
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] optimized_estimates applied sender_id=%u origin_robot_id=%u estimates=%zu",
        msg->sender_id, origin_robot_id_, msg->estimates.size());
  }
  else
  {
    RCLCPP_WARN(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] optimized_estimates ignored odom_values=%lu estimates=%zu",
        odometry_pose_estimates_->size(), msg->estimates.size());
  }
}

void DecentralizedPGO::share_optimized_estimates(
    const gtsam::Values &estimates)
{
  if (enable_simulated_rendezvous_ && !sim_rdv_->is_alive())
  {
    return;
  }
  auto is_pose_graph_connected = connected_robot_pose_graph();
  auto included_robots_ids = current_neighbors_ids_;
  included_robots_ids.robots.ids.push_back(robot_id_);
  for (unsigned int i = 0; i < included_robots_ids.robots.ids.size(); i++)
  {
    cslam_common_interfaces::msg::OptimizationResult msg;
    msg.success = true;
    msg.sender_id = robot_id_;
    msg.origin_robot_id = origin_robot_id_;
    const unsigned int target_robot_id = included_robots_ids.robots.ids[i];
    const bool share_full_component =
        target_robot_id == robot_id_ || is_pose_graph_connected[target_robot_id];
    if (share_full_component)
    {
      msg.estimates = gtsam_values_to_msg(estimates);
    }
    else
    {
      auto filtered_estimates =
          estimates.filter(gtsam::LabeledSymbol::LabelTest(
              ROBOT_LABEL(target_robot_id)));
      msg.estimates = gtsam_values_to_msg(filtered_estimates);
    }
    optimized_estimates_publishers_[included_robots_ids.robots.ids[i]]->publish(
        msg);
    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] share_optimized_estimates target_robot=%u sender_id=%u origin_robot_id=%u shared_estimates=%zu total_estimates=%zu full_component=%s",
        target_robot_id, msg.sender_id, origin_robot_id_,
        msg.estimates.size(), estimates.size(),
        share_full_component ? "true" : "false");
  }
}

void DecentralizedPGO::publish_merged_visualization_pose_graph(
    const gtsam::Values &estimates)
{
  auto current_values = gtsam_values_to_msg(estimates);
  auto current_edges = gtsam_factors_to_msg(aggregate_pose_graph_.first);
  std::set<unsigned int> robots_in_current_result;
  std::set<std::pair<unsigned int, unsigned int>> edge_pairs_in_current_result;

  for (const auto &value : current_values)
  {
    robots_in_current_result.insert(value.key.robot_id);
  }

  for (auto robot_id : robots_in_current_result)
  {
    merged_visualization_values_by_robot_[robot_id].clear();
  }

  for (const auto &value : current_values)
  {
    merged_visualization_values_by_robot_[value.key.robot_id].push_back(value);
  }

  for (const auto &edge : current_edges)
  {
    unsigned int robot0_id = edge.key_from.robot_id;
    unsigned int robot1_id = edge.key_to.robot_id;
    auto robot_pair = std::make_pair(
        std::min(robot0_id, robot1_id),
        std::max(robot0_id, robot1_id));
    edge_pairs_in_current_result.insert(robot_pair);
  }

  for (const auto &robot_pair : edge_pairs_in_current_result)
  {
    merged_visualization_edges_by_robot_pair_[robot_pair].clear();
  }

  for (const auto &edge : current_edges)
  {
    unsigned int robot0_id = edge.key_from.robot_id;
    unsigned int robot1_id = edge.key_to.robot_id;
    auto robot_pair = std::make_pair(
        std::min(robot0_id, robot1_id),
        std::max(robot0_id, robot1_id));
    merged_visualization_edges_by_robot_pair_[robot_pair].push_back(edge);
  }

  if (merged_visualization_pose_graph_publisher_->get_subscription_count() == 0)
  {
    return;
  }

  cslam_common_interfaces::msg::PoseGraph out_msg;
  out_msg.robot_id = robot_id_;
  out_msg.origin_robot_id = origin_robot_id_;

  for (const auto &entry : merged_visualization_values_by_robot_)
  {
    out_msg.connected_robots.ids.push_back(entry.first);
    out_msg.values.insert(
        out_msg.values.end(),
        entry.second.begin(),
        entry.second.end());
  }

  for (const auto &entry : merged_visualization_edges_by_robot_pair_)
  {
    out_msg.edges.insert(
        out_msg.edges.end(),
        entry.second.begin(),
        entry.second.end());
  }

  merged_visualization_pose_graph_publisher_->publish(out_msg);

  RCLCPP_INFO(
      node_->get_logger(),
      "[DEBUG_BACKEND_PIPELINE] merged visualization graph published source_robot=%u origin_robot_id=%u cached_robots=%zu values=%zu edges=%zu",
      robot_id_, origin_robot_id_,
      merged_visualization_values_by_robot_.size(),
      out_msg.values.size(),
      out_msg.edges.size());
}

void DecentralizedPGO::heartbeat_timer_callback()
{
  if (enable_simulated_rendezvous_)
  {
    if (!sim_rdv_->is_alive()) {
      return;
    }
  }
  std_msgs::msg::UInt32 msg;
  msg.data = origin_robot_id_;
  heartbeat_publisher_->publish(msg);
}

void DecentralizedPGO::visualization_callback()
{
  if (visualization_pose_graph_publisher_->get_subscription_count() > 0)
  {
    cslam_common_interfaces::msg::PoseGraph out_msg;
    out_msg.robot_id = robot_id_;
    out_msg.origin_robot_id = origin_robot_id_;
    auto local_current_estimates =
        pose_values_for_robot(*current_pose_estimates_, robot_id_);
    if (local_current_estimates->size() >= odometry_pose_estimates_->size())
    {
      out_msg.values = gtsam_values_to_msg(local_current_estimates);
    }
    else
    {
      out_msg.values = gtsam_values_to_msg(odometry_pose_estimates_);
    }
    auto graph = boost::make_shared<gtsam::NonlinearFactorGraph>();
    graph->push_back(pose_graph_->begin(), pose_graph_->end());

    for (unsigned int i = 0; i < max_nb_robots_; i++)
    {
      for (unsigned int j = i + 1; j < max_nb_robots_; j++)
      {
        unsigned int min_robot_id = std::min(i, j);
        unsigned int max_robot_id = std::max(i, j);
        if (inter_robot_loop_closures_[{min_robot_id, max_robot_id}].size() > 0 &&
            (min_robot_id == robot_id_ || max_robot_id == robot_id_))
        {
          if (min_robot_id == robot_id_)
          {
            graph->push_back(
                inter_robot_loop_closures_[{min_robot_id, max_robot_id}].begin(),
                inter_robot_loop_closures_[{min_robot_id, max_robot_id}].end());
          }
        }
      }
    }

    out_msg.edges = gtsam_factors_to_msg(graph);
    visualization_pose_graph_publisher_->publish(out_msg);
  }
}

void DecentralizedPGO::update_transform_to_origin(const gtsam::Pose3 &pose)
{
  rclcpp::Time now = node_->get_clock()->now();
  origin_to_first_pose_.header.stamp = now;
  origin_to_first_pose_.header.frame_id = MAP_FRAME_ID(origin_robot_id_);
  origin_to_first_pose_.child_frame_id = MAP_FRAME_ID(robot_id_);

  origin_to_first_pose_.transform = gtsam_pose_to_transform_msg(pose);

  // Update the reference frame
  // This is the key info for many tasks since it allows conversions from
  // one robot reference frame to another.
  if (reference_frame_per_robot_publisher_->get_subscription_count() > 0)
  {
    cslam_common_interfaces::msg::ReferenceFrames msg;
    msg.robot_id = robot_id_;
    msg.origin_to_local = origin_to_first_pose_;
    reference_frame_per_robot_publisher_->publish(msg);
  }
  // Store for TF
  local_pose_at_latest_optimization_ = tentative_local_pose_at_latest_optimization_;
  local_symbol_at_latest_optimization_ = tentative_local_symbol_at_latest_optimization_;
  if (local_symbol_at_latest_optimization_ != gtsam::LabeledSymbol() &&
      current_pose_estimates_->exists(local_symbol_at_latest_optimization_))
  {
    latest_optimized_pose_ =
        current_pose_estimates_->at<gtsam::Pose3>(local_symbol_at_latest_optimization_);
  }
  else if (!current_pose_estimates_->empty())
  {
    latest_optimized_pose_ =
        current_pose_estimates_->at<gtsam::Pose3>(current_pose_estimates_->keys().back());
    RCLCPP_WARN(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] latest optimized symbol missing; falling back to highest key.");
  }
}

void DecentralizedPGO::broadcast_tf_callback()
{
  static uint64_t tf_pub_count = 0;
  tf_pub_count++;

  if (odometry_pose_estimates_->empty() && (tf_pub_count <= 5 || tf_pub_count % 50 == 0))
  {
    RCLCPP_WARN(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] no keyframe odometry received yet; pose output may stay at default.");
  }

  // Useful for visualization.
  // For tasks purposes you might want to use reference_frame_per_robot_ instead
  // Since it is updated only when a new optimization is performed.

  // origin to local map
  rclcpp::Time now = node_->get_clock()->now();
  origin_to_first_pose_.header.stamp = now;
  if (origin_to_first_pose_.header.frame_id !=
      origin_to_first_pose_.child_frame_id)
  {
    tf_broadcaster_->sendTransform(origin_to_first_pose_);
  }

  // origin to latest optimized pose
  geometry_msgs::msg::TransformStamped latest_optimized_pose_msg;
  latest_optimized_pose_msg.header.stamp = now;
  latest_optimized_pose_msg.header.frame_id = MAP_FRAME_ID(origin_robot_id_);
  latest_optimized_pose_msg.child_frame_id = LATEST_OPTIMIZED_FRAME_ID(robot_id_);
  latest_optimized_pose_msg.transform = gtsam_pose_to_transform_msg(
        latest_optimized_pose_);
  tf_broadcaster_->sendTransform(latest_optimized_pose_msg);

  // latest optimized pose to latest local pose (odometry alone)
  geometry_msgs::msg::TransformStamped current_transform_msg;
  current_transform_msg.header.stamp = now;
  current_transform_msg.header.frame_id = LATEST_OPTIMIZED_FRAME_ID(robot_id_);
  current_transform_msg.child_frame_id = CURRENT_FRAME_ID(robot_id_);
  gtsam::Pose3 current_pose_diff = local_pose_at_latest_optimization_.inverse() * latest_local_pose_;
  current_transform_msg.transform = gtsam_pose_to_transform_msg(current_pose_diff);
  tf_broadcaster_->sendTransform(current_transform_msg);

  // Publish as message latest estimate (optimized pose + odometry)
  geometry_msgs::msg::PoseStamped pose_msg;
  if (latest_local_pose_stamp_.sec != 0 ||
      latest_local_pose_stamp_.nanosec != 0)
  {
    pose_msg.header.stamp = latest_local_pose_stamp_;
  }
  else
  {
    pose_msg.header.stamp = now;
  }
  pose_msg.header.frame_id = MAP_FRAME_ID(origin_robot_id_);
  pose_msg.pose = gtsam_pose_to_msg(latest_optimized_pose_ * current_pose_diff);
  optimized_pose_estimate_publisher_->publish(pose_msg);

  if (tf_pub_count <= 5 || tf_pub_count % 50 == 0)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] pose published topic=/r%u/cslam/current_pose_estimate frame=%s pos=(%.3f, %.3f, %.3f)",
        robot_id_, pose_msg.header.frame_id.c_str(),
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  }
}

gtsam::Values
DecentralizedPGO::optimize(const gtsam::NonlinearFactorGraph::shared_ptr &graph,
                           const gtsam::Values::shared_ptr &initial)
{
  gtsam::Values result;
  if (enable_logs_){
    logger_->start_timer();
  }
  try{
    gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
    gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
        optimizer(*graph, *initial, params);
    result = optimizer.optimize();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Optimization failed: %s", e.what());
    result = *initial;
  }
  if (enable_logs_){
    logger_->stop_timer();
    try{
      logger_->log_optimized_global_pose_graph(graph, result, robot_id_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Logging failed: %s", e.what());
      result = *initial;
    }
  }
  return result;
}

void DecentralizedPGO::start_optimization()
{
  // Build global pose graph
  aggregate_pose_graph_ = aggregate_pose_graphs();

  // Add prior
  // Use the initial local anchor pose. Under normal startup this is keyframe 0,
  // but we keep a fallback when the very first message was missed.
  gtsam::LabeledSymbol first_symbol = anchor_symbol_;

  if (first_symbol == gtsam::LabeledSymbol() ||
      !current_pose_estimates_->exists(first_symbol))
  {
    RCLCPP_WARN(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] optimization skipped: anchor symbol missing. anchor_initialized=%s current_values=%lu odom_values=%lu",
        first_symbol == gtsam::LabeledSymbol() ? "false" : "true",
        current_pose_estimates_->size(), odometry_pose_estimates_->size());
    return;
  }

  aggregate_pose_graph_.first->addPrior(
      first_symbol, current_pose_estimates_->at<gtsam::Pose3>(first_symbol),
      default_noise_model_);

  if (enable_logs_){
    logger_->log_initial_global_pose_graph(aggregate_pose_graph_.first, aggregate_pose_graph_.second);
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[DEBUG_BACKEND_PIPELINE] starting optimization factors=%zu estimates=%zu",
      aggregate_pose_graph_.first->size(), aggregate_pose_graph_.second->size());

  // Optimize graph
  optimization_result_ =
      std::async(&DecentralizedPGO::optimize, this, aggregate_pose_graph_.first,
                 aggregate_pose_graph_.second);
  optimizer_state_ = OptimizerState::OPTIMIZATION;
}

void DecentralizedPGO::check_result_and_finish_optimization()
{
  auto status = optimization_result_.wait_for(std::chrono::milliseconds(0));

  if (status == std::future_status::ready)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Pose Graph Optimization completed.");
    auto result = optimization_result_.get();
    optimization_count_++;

    // Share results
    publish_merged_visualization_pose_graph(result);
    share_optimized_estimates(result);
    optimizer_state_ = OptimizerState::IDLE;

    // Publish result info for monitoring
    if (debug_optimization_result_publisher_->get_subscription_count() > 0)
    {
      cslam_common_interfaces::msg::OptimizationResult msg;
      msg.success = true;
      msg.sender_id = robot_id_;
      msg.origin_robot_id = origin_robot_id_;
      msg.factors = gtsam_factors_to_msg(aggregate_pose_graph_.first);
      msg.estimates = gtsam_values_to_msg(result);
      debug_optimization_result_publisher_->publish(msg);
    }
  }
}

void DecentralizedPGO::optimization_loop_callback()
{
  static int previous_state = -1;
  if (previous_state != static_cast<int>(optimizer_state_))
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "[DEBUG_BACKEND_PIPELINE] optimizer state -> %s",
        optimizer_state_to_string(optimizer_state_));
    previous_state = static_cast<int>(optimizer_state_);
  }

  const bool rendezvous_active =
      !(enable_simulated_rendezvous_ && !sim_rdv_->is_alive());
  if (!rendezvous_active)
  {
    if (optimizer_state_ != OptimizerState::IDLE)
    {
      end_waiting();
      optimizer_state_ = OptimizerState::IDLE;
      reinitialize_received_pose_graphs();
      other_robots_graph_and_estimates_.clear();
    }
  }
  else if (!odometry_pose_estimates_->empty())
  {
    if (optimizer_state_ ==
        OptimizerState::POSEGRAPH_COLLECTION)
    {
      if (current_neighbors_ids_.robots.ids.size() > 0)
      {
        for (auto id : current_neighbors_ids_.robots.ids)
        {
          auto current_robots_ids = current_neighbors_ids_;
          current_robots_ids.robots.ids.push_back(robot_id_);
          get_pose_graph_publishers_[id]->publish(current_robots_ids.robots);
        }
        start_waiting();
      }
      else
      {
        optimizer_state_ = OptimizerState::START_OPTIMIZATION;
      }
    }
    else if (optimizer_state_ == OptimizerState::START_OPTIMIZATION)
    {
      // Call optimization
      start_optimization();
    }
    else if (optimizer_state_ == OptimizerState::OPTIMIZATION)
    {
      check_result_and_finish_optimization();
    }
    else if (is_waiting())
    {
      check_waiting_timeout();
    }
  }
  if (optimizer_state_publisher_->get_subscription_count() > 0)
  {
    cslam_common_interfaces::msg::OptimizerState state_msg;
    state_msg.state = optimizer_state_;
    optimizer_state_publisher_->publish(state_msg);
  }
}
