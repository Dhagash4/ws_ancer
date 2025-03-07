#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include <map>

namespace trajectory_io {

// Read Trajectory data from file
std::vector<geometry_msgs::msg::Pose>
readTrajectoryData(const std::string &filepath);

// Save Trajectory data for specified duration to file
bool saveTrajectoryData(
    const std::string &filename, const std::string &format,
    const std::map<rclcpp::Time, geometry_msgs::msg::Pose> &data,
    const rclcpp::Time &start_time);

} // namespace trajectory_io
