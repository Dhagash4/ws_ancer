#include "anscer_trajectory_utils/trajectory_reader_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace anscer_trajectory_utils {

TrajectoryReaderNode::TrajectoryReaderNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("trajectory_reader_node", options),
      pub_topic_name_("/anscer_reader_trajectory_publisher") {
  bool onConfigure = configureParameters();
  if (onConfigure) {
    configurePubSub();
    // Transform buffer and listener initialization
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  } else {
    RCLCPP_DEBUG(get_logger(),
                 "Shutting down the node, as parameters configuration failed");
    rclcpp::shutdown();
  }
  this->publishTrajMarkerArray();
}

TrajectoryReaderNode::~TrajectoryReaderNode() {}

bool TrajectoryReaderNode::configureParameters() {
  declare_parameter("filepath", "");
  declare_parameter("base_frame_id", "base_footprint");
  declare_parameter("map_frame_id", "odom");

  std::string filepath_;

  if (!get_parameter("filepath", filepath_)) {
    RCLCPP_ERROR(get_logger(), "Failed to get parameter 'topic'");
    return false;
  }
  filepath_ = get_parameter("filepath").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  map_frame_id_ = get_parameter("map_frame_id").as_string();

  bool readData = this->readTrajectoryData(filepath_);

  if (!readData) {
    return false;
  }

  return true;
}

bool TrajectoryReaderNode::readTrajectoryData(const std::string &filepath) {

  trajectory_data_ = trajectory_io::readTrajectoryData(filepath);

  if (trajectory_data_.empty()) {
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "Data reading is completed %ld",
               trajectory_data_.size());
  return true;
}

void TrajectoryReaderNode::configurePubSub() {
  // Set QoS settings
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  trajectory_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic_name_,
                                                             qos);
}

void TrajectoryReaderNode::publishTrajMarkerArray() {

  // Try to get the latest transform from base_footprint to odom.
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    // Lookup transform: target frame map_frame, source frame base_frame
    // using tf2::TimePointZero to get the latest available transform.
    transformStamped = tf_buffer_->lookupTransform(
        base_frame_id_,           // target frame
        map_frame_id_,            // source frame
        tf2::TimePointZero,       // time
        tf2::durationFromSec(0.5) // tolerance duration
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(),
                "Could not get transform from base_footprint to odom: %s",
                ex.what());
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  for (const auto &pose : trajectory_data_) {
    // Wrap the pose (assumed in base_footprint frame) in a PoseStamped message.
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header.frame_id = base_frame_id_;
    pose_in.header.stamp =
        this->now(); // Use current time or a specific timestamp if available.
    pose_in.pose = pose;

    // Transform the pose from base_footprint to odom frame.
    tf2::doTransform(pose_in, pose_out, transformStamped);

    // Create a marker for the transformed pose.
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_; // Publish marker in odom frame.
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose_out.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_array.markers.push_back(marker);
    RCLCPP_DEBUG(get_logger(), "Should start publishing now %d", id);
  }

  // Publish the MarkerArray.
  trajectory_publisher_->publish(marker_array);
}
} // namespace anscer_trajectory_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(anscer_trajectory_utils::TrajectoryReaderNode)