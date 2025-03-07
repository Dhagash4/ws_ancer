
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

def generate_launch_description():
    # Get the launch directory
    anscer_trajectory_utils_dir = get_package_share_directory("anscer_trajectory_utils")
    config = os.path.join(anscer_trajectory_utils_dir, "config", "trajectory_reader_params.yaml")

    params_file = LaunchConfiguration("params_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=config,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    static_transform_node =Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_1",
                output="screen",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "1.0",
                    "odom",
                    "base_footprint",
                ],
            )


   

    trajectory_reader_node = Node(
        package="anscer_trajectory_utils",
        executable="trajectory_reader_node",
        name="trajectory_reader_node",
        parameters=[params_file],
        output="screen",  
        arguments=[
            "--ros-args",
            "--log-level",
            "DEBUG",
            "--log-level",
            "rcl:=INFO",
        ],  
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    # ld.add_action(static_transform_node)
  
    ld.add_action(trajectory_reader_node)

    return ld
