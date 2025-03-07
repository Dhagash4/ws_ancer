#include "anscer_trajectory_utils/trajectory_storage_node.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <filesystem>

namespace fs = std::filesystem;

namespace anscer_trajectory_utils {

TrajectoryStorageNode::TrajectoryStorageNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("trajectory_saver_node", options),
      pub_topic_name_("/anscer_trajectory_visualizer") {
  bool onConfigure = configureParameters();

  if (onConfigure) {
    configurePubSub();
    configureServices();
  } else {
    RCLCPP_DEBUG(get_logger(),
                 "Shutting down the node, as parameters configuration failed");
    rclcpp::shutdown();
  }
}

TrajectoryStorageNode::~TrajectoryStorageNode() {
  RCLCPP_DEBUG(get_logger(), "Shutting down the node gracefully");
}

bool TrajectoryStorageNode::configureParameters() {
  // Declare the parameters
  declare_parameter("topic", "/odom");

  if (!get_parameter("topic", sub_topic_name_)) {
    RCLCPP_ERROR(get_logger(), "Failed to get parameter 'topic'");
    return false;
  }
  // Get the parameter
  sub_topic_name_ = get_parameter("topic").as_string();
  return true;
}

void TrajectoryStorageNode::configurePubSub() {
  // Set QoS settings
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  trajectory_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic_name_,
                                                             qos);
  trajectory_subsctiption_ = create_subscription<nav_msgs::msg::Odometry>(
      sub_topic_name_, qos,
      std::bind(&TrajectoryStorageNode::trajectorySubCallback, this,
                std::placeholders::_1));
}

void TrajectoryStorageNode::configureServices() {
  trajectory_saver_service_ = create_service<anscer_msgs::srv::SaveTrajectory>(
      "save_trajectory",
      std::bind(&TrajectoryStorageNode::saveTrajectory, this,
                std::placeholders::_1, std::placeholders::_2));
}

void TrajectoryStorageNode::trajectorySubCallback(
    nav_msgs::msg::Odometry::ConstSharedPtr traj_msg) {
  auto time = this->now();
  const auto pose = traj_msg->pose.pose;
  const auto frame_id = traj_msg->header.frame_id;
  writeTrajectoryData(time, pose);
  publishTrajMarkerArray(frame_id);
}

void TrajectoryStorageNode::publishTrajMarkerArray(
    const std::string &frame_id) {
  auto marker_array = visualization_msgs::msg::MarkerArray();
  int id = 0;

  for (const auto &pair : trajectory_data_) {
    const auto &pose = pair.second;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;

    marker.scale.x = 0.1; // Shaft length
    marker.scale.y = 0.1; // Shaft diameter
    marker.scale.z = 0.1; // Head diameter

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker_array.markers.push_back(marker);
    id += 1;
  }

  trajectory_publisher_->publish(marker_array);
  RCLCPP_INFO(get_logger(), "Published %zu markers",
              marker_array.markers.size());
}

void TrajectoryStorageNode::saveTrajectory(
    const std::shared_ptr<anscer_msgs::srv::SaveTrajectory::Request> request,
    std::shared_ptr<anscer_msgs::srv::SaveTrajectory::Response> response) {
  fs::path filename(request->filename);

  // Extract and normalize the file extension (format)
  std::string extension = filename.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);

  if (extension != ".csv" && extension != ".json" && extension != ".yaml") {
    RCLCPP_ERROR(get_logger(), "Unsupported file format: %s",
                 extension.c_str());
    response->message = "Unsupportted file format";
    return;
  }

  auto duration_sec = request->duration;
  rclcpp::Time start_time =
      now() - rclcpp::Duration::from_seconds(duration_sec);

  bool success = trajectory_io::saveTrajectoryData(
      filename.string(), extension, trajectory_data_, start_time);

  if (success) {
    RCLCPP_INFO(get_logger(),
                "Trajectory Data saved successfully with %s filename",
                filename.string().c_str());
    response->message = "Service finished successfully";
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to save trajectory data.");
    response->message = "Service failed";
  }
}

} // namespace anscer_trajectory_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(anscer_trajectory_utils::TrajectoryStorageNode)