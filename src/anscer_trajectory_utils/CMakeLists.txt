cmake_minimum_required(VERSION 3.8)
project(anscer_trajectory_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(anscer_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    anscer_msgs
)

set(io_lib_dependencies
    rclcpp
    geometry_msgs
    nlohmann_json
    yaml-cpp
)

# Library and node names
set(io_lib trajectory_io)
set(storage_lib trajectory_storage_lib)
set(storage_node trajectory_storage_node)
set(reader_lib trajectory_reader_lib)
set(reader_node trajectory_reader_node)

# Include directory for headers
include_directories(include)

# Build the trajectory_io shared library.
add_library(${io_lib} SHARED src/trajectory_io.cpp)
target_link_libraries(${io_lib} yaml-cpp nlohmann_json)
ament_target_dependencies(${io_lib} ${io_lib_dependencies})

# Build the trajectory_storage_node shared library.
add_library(${storage_lib} SHARED src/trajectory_storage_node.cpp)
ament_target_dependencies(${storage_lib} ${dependencies})
target_link_libraries(${storage_lib} ${io_lib})

# Build the trajectory_reader_node shared library.
add_library(${reader_lib} SHARED src/trajectory_reader_node.cpp)
target_link_libraries(${reader_lib} ${io_lib} tf2_geometry_msgs::tf2_geometry_msgs)
ament_target_dependencies(${reader_lib} ${dependencies})

# Register component nodes.
rclcpp_components_register_node(${storage_lib}
  PLUGIN "anscer_trajectory_utils::TrajectoryStorageNode"
  EXECUTABLE ${storage_node}
)

rclcpp_components_register_node(${reader_lib}
  PLUGIN "anscer_trajectory_utils::TrajectoryReaderNode"
  EXECUTABLE ${reader_node}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for copyrights and cpplint if not applicable
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install shared libraries.
install(TARGETS ${storage_lib} ${reader_lib} ${io_lib}
        ARCHIVE DESTINATION lib 
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install executables for components.
install(TARGETS ${storage_node} ${reader_node}
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
