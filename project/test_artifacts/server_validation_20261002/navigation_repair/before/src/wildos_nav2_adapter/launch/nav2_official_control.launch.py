"""Nav2 local execution for the official WildOS /spot1/goal_pose output."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("wildos_nav2_adapter"),
                          "config", "nav2_official_control.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        Node(package="wildos_nav2_adapter", executable="cloud_filter",
             name="wildos_official_cloud_filter",
             parameters=[{"input_topic": "/spot1/ouster/front/points_filtered",
                          "output_topic": "/wildos/points",
                          "clearing_topic": "/wildos/clearing_points",
                          "output_frame": "spot1/base_link",
                          "footprint_front_extent": 3.4,
                          "footprint_rear_extent": 1.05,
                          "footprint_half_width": 1.05,
                          "voxel": 0.2, "max_points": 20000, "max_range": 40.0,
                          "min_height": 0.1, "max_height": 3.0,
                          "ground_plane_radius": 8.0,
                          "ground_clearance": 0.25}],
             output="screen"),
        # beamng_wildos_adapter owns spot1/odom -> spot1/base_link TF.
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[LaunchConfiguration("params_file")],
             remappings=[("cmd_vel", "/wildos/cmd_vel")], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_official",
             parameters=[{"autostart": True,
                          "node_names": ["planner_server", "controller_server"]}],
             output="screen"),
    ])
