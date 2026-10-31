"""Nav2 planner + controller with rolling LiDAR costmaps.

Produces /wildos/cmd_vel. Actual vehicle actuation is gated by the separate
cmd_vel_to_control adapter (disabled unless enable:=true).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("wildos_nav2_adapter"),
        "config",
        "nav2_control.yaml",
    )
    params_file = LaunchConfiguration("params_file")
    input_cloud = LaunchConfiguration("input_cloud")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("input_cloud", default_value="/vehicles/ego/sensors/front_lidar"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="wildos_map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        Node(
            package="wildos_nav2_adapter",
            executable="cloud_filter",
            name="wildos_cloud_filter",
            parameters=[{
                "input_topic": input_cloud,
                "output_topic": "/wildos/points",
                "clearing_topic": "/wildos/clearing_points",
                "voxel": 0.2,
                "max_points": 20000,
                "max_range": 40.0,
                "min_height": 0.1,
                "max_height": 3.0,
                "output_frame": "base_link",
            }],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[params_file],
            remappings=[("cmd_vel", "/wildos/cmd_vel")],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_control",
            parameters=[{
                "autostart": True,
                "node_names": ["planner_server", "controller_server"],
            }],
            output="screen",
        ),
    ])
