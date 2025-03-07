#pragma once

#include "anscer_trajectory_utils/trajectory_io.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace anscer_trajectory_utils {
class TrajectoryReaderNode : public rclcpp::Node {
public:
  explicit TrajectoryReaderNode(const rclcpp::NodeOptions &options);
  ~TrajectoryReaderNode();

private:
  bool configureParameters();

  void configurePubSub();

  // Read the trajectory data from the filepath
  bool readTrajectoryData(const std::string &filepath);

  // Convert to marker array and publish
  void publishTrajMarkerArray();

  // Publisher to visualize trajectory of the robot
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      trajectory_publisher_;

  // ROS2 Variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Trajectory data
  std::vector<geometry_msgs::msg::Pose> trajectory_data_;

  // Base frame_id and Map frame_id
  std::string base_frame_id_;
  std::string map_frame_id_;
  std::string pub_topic_name_;
};
} // namespace anscer_trajectory_utils