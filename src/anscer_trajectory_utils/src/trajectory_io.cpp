#include "anscer_trajectory_utils/trajectory_io.hpp"
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace trajectory_io {

std::vector<geometry_msgs::msg::Pose>
readTrajectoryData(const std::string &filepath) {

  std::vector<geometry_msgs::msg::Pose> poses;

  // Open the file for reading.
  std::ifstream ifs(filepath);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("Trajectory"), "Failed to open file: %s",
                 filepath.c_str());
    return poses;
  }

  // Determine the file format by its extension.
  fs::path file_path(filepath);
  std::string ext = file_path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  if (ext == ".csv") {
    // CSV Format: Expected header "time,px,py,pz,ox,oy,oz,ow"
    std::string line;
    // Read and discard header line.
    if (!std::getline(ifs, line)) {
      RCLCPP_ERROR(rclcpp::get_logger("Trajectory"), "Empty CSV file: %s",
                   filepath.c_str());
      return poses;
    }
    // Process each subsequent line.
    while (std::getline(ifs, line)) {
      if (line.empty()) {
        continue;
      }
      std::stringstream ss(line);
      std::string token;
      std::vector<std::string> tokens;
      // Split the line by comma.
      while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
      }
      // Check we have enough tokens (time, and 7 pose fields).
      if (tokens.size() < 8) {
        RCLCPP_WARN(rclcpp::get_logger("Trajectory"),
                    "Invalid CSV line in file: %s", filepath.c_str());
        continue;
      }
      geometry_msgs::msg::Pose pose;
      // tokens[0] is the time stamp (ignored here).
      try {
        pose.position.x = std::stod(tokens[1]);
        pose.position.y = std::stod(tokens[2]);
        pose.position.z = std::stod(tokens[3]);
        pose.orientation.x = std::stod(tokens[4]);
        pose.orientation.y = std::stod(tokens[5]);
        pose.orientation.z = std::stod(tokens[6]);
        pose.orientation.w = std::stod(tokens[7]);
      } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("Trajectory"),
                    "Error parsing CSV line: %s", e.what());
        continue;
      }
      poses.push_back(pose);
    }
  } else if (ext == ".json") {
    // JSON Format: Expected a JSON array of objects where each object
    // contains: "time", "position": {"x", "y", "z"}, and "orientation": {"x",
    // "y", "z", "w"}
    try {
      nlohmann::json j;
      ifs >> j;
      for (const auto &item : j) {
        geometry_msgs::msg::Pose pose;
        // We ignore the "time" field.
        auto pos = item["position"];
        auto orient = item["orientation"];
        pose.position.x = pos["x"].get<double>();
        pose.position.y = pos["y"].get<double>();
        pose.position.z = pos["z"].get<double>();
        pose.orientation.x = orient["x"].get<double>();
        pose.orientation.y = orient["y"].get<double>();
        pose.orientation.z = orient["z"].get<double>();
        pose.orientation.w = orient["w"].get<double>();
        poses.push_back(pose);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("Trajectory"), "JSON parse error: %s",
                   e.what());
      return std::vector<geometry_msgs::msg::Pose>();
    }
  } else if (ext == ".yaml") {
    // YAML Format: Expected a YAML sequence of entries with:
    // - time: <value>
    //   position:
    //     x: <value>
    //     y: <value>
    //     z: <value>
    //   orientation:
    //     x: <value>
    //     y: <value>
    //     z: <value>
    //     w: <value>
    try {
      YAML::Node yaml_node = YAML::Load(ifs);
      for (const auto &item : yaml_node) {
        geometry_msgs::msg::Pose pose;
        // "time" is ignored.
        YAML::Node pos = item["position"];
        YAML::Node orient = item["orientation"];
        pose.position.x = pos["x"].as<double>();
        pose.position.y = pos["y"].as<double>();
        pose.position.z = pos["z"].as<double>();
        pose.orientation.x = orient["x"].as<double>();
        pose.orientation.y = orient["y"].as<double>();
        pose.orientation.z = orient["z"].as<double>();
        pose.orientation.w = orient["w"].as<double>();
        poses.push_back(pose);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("Trajectory"), "YAML parse error: %s",
                   e.what());
      return std::vector<geometry_msgs::msg::Pose>();
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Trajectory"),
                 "Unsupported file format: %s", ext.c_str());
  }

  ifs.close();
  return poses;
}

bool saveTrajectoryData(
    const std::string &filename, const std::string &format,
    const std::map<rclcpp::Time, geometry_msgs::msg::Pose> &data,
    const rclcpp::Time &start_time) {

  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("Trajectory"), "Failed to open file: %s",
                 filename.c_str());
    return false;
  }

  if (format == ".csv") {
    ofs << "time,px,py,pz,ox,oy,oz,ow\n";
    // Iterate through the map from start_time to the end.
    for (auto it = data.lower_bound(start_time); it != data.end(); ++it) {
      const auto &time_point = it->first;
      const auto &pose = it->second;
      ofs << time_point.seconds() << "," << pose.position.x << ","
          << pose.position.y << "," << pose.position.z << ","
          << pose.orientation.x << "," << pose.orientation.y << ","
          << pose.orientation.z << "," << pose.orientation.w << "\n";
    }
  } else if (format == ".json") {
    ofs << "[\n";
    bool first = true;
    for (auto it = data.lower_bound(start_time); it != data.end(); ++it) {
      if (!first) {
        ofs << ",\n";
      }
      first = false;
      const auto &time_point = it->first;
      const auto &pose = it->second;
      ofs << "  {\n"
          << "    \"time\": " << time_point.seconds() << ",\n"
          << "    \"position\": {\"x\": " << pose.position.x
          << ", \"y\": " << pose.position.y << ", \"z\": " << pose.position.z
          << "},\n"
          << "    \"orientation\": {\"x\": " << pose.orientation.x
          << ", \"y\": " << pose.orientation.y
          << ", \"z\": " << pose.orientation.z
          << ", \"w\": " << pose.orientation.w << "}\n"
          << "  }";
    }
    ofs << "\n]\n";
  } else if (format == ".yaml") {
    // Output YAML formatted entries.
    for (auto it = data.lower_bound(start_time); it != data.end(); ++it) {
      const auto &time_point = it->first;
      const auto &pose = it->second;
      ofs << "- time: " << time_point.seconds() << "\n"
          << "  position:\n"
          << "    x: " << pose.position.x << "\n"
          << "    y: " << pose.position.y << "\n"
          << "    z: " << pose.position.z << "\n"
          << "  orientation:\n"
          << "    x: " << pose.orientation.x << "\n"
          << "    y: " << pose.orientation.y << "\n"
          << "    z: " << pose.orientation.z << "\n"
          << "    w: " << pose.orientation.w << "\n";
    }
  } else {
    // Unsupported format.
    ofs.close();
    return false;
  }

  ofs.close();
  return true;
}

} // namespace trajectory_io
