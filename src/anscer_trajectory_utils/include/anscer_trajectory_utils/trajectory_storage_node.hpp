#pragma once
#include <memory>

#include "anscer_msgs/srv/save_trajectory.hpp"
#include "anscer_trajectory_utils/trajectory_io.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace anscer_trajectory_utils {
class TrajectoryStorageNode : public rclcpp::Node {
public:
  // Constructor
  explicit TrajectoryStorageNode(const rclcpp::NodeOptions &options);

  // Destructor
  ~TrajectoryStorageNode();

private:
  // Configure parameters
  bool configureParameters();

  // Configure pub/sub
  void configurePubSub();

  // Configure Service
  void configureServices();

  inline void writeTrajectoryData(const rclcpp::Time &time,
                                  const geometry_msgs::msg::Pose &pose) {
    trajectory_data_.emplace(time, pose);
  }

  // Convert to marker array and publish
  void publishTrajMarkerArray(const std::string &frame_id);

  // Subscriber callback functions
  void trajectorySubCallback(nav_msgs::msg::Odometry::ConstSharedPtr traj_msg);

  // Subscribe to robot trajectory publisher topic
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      trajectory_subsctiption_;

  // Publisher to visualize trajectory of the robot
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      trajectory_publisher_;

  // Service for saving the trajectory
  rclcpp::Service<anscer_msgs::srv::SaveTrajectory>::SharedPtr
      trajectory_saver_service_;

  // Service callback functions
  void saveTrajectory(
      const std::shared_ptr<anscer_msgs::srv::SaveTrajectory::Request> request,
      std::shared_ptr<anscer_msgs::srv::SaveTrajectory::Response> response);

  // Variables
  std::string sub_topic_name_;
  std::string pub_topic_name_;

  // Trajectory Data
  std::map<rclcpp::Time, geometry_msgs::msg::Pose> trajectory_data_;
};
} // namespace anscer_trajectory_utils