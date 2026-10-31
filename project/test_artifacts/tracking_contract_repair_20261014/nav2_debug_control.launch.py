"""Nav2 local execution for the official WildOS /spot1/goal_pose output."""
import os
from wildos_nav2_adapter.vehicle_geometry import SBR_OBSTACLE_MAX_HEIGHT
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("wildos_nav2_adapter"),
                          "config", "nav2_official_control.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        # The default carrier has no body. Physical-car demos must explicitly
        # restore their measured SBR extents and self-return filtering.
        DeclareLaunchArgument("footprint_front_extent", default_value="0.0"),
        DeclareLaunchArgument("footprint_rear_extent", default_value="0.0"),
        DeclareLaunchArgument("footprint_half_width", default_value="0.0"),
        DeclareLaunchArgument("filter_footprint_returns", default_value="false"),
        Node(package="wildos_nav2_adapter", executable="cloud_filter",
             name="wildos_official_cloud_filter",
             parameters=[{"input_topic": "/spot1/ouster/front/points_filtered",
                          "output_topic": "/wildos/points",
                          "clearing_topic": "/wildos/clearing_points",
                          "output_frame": "spot1/base_link",
                          "filter_footprint_returns": ParameterValue(
                              LaunchConfiguration("filter_footprint_returns"), value_type=bool),
                          "footprint_front_extent": ParameterValue(
                              LaunchConfiguration("footprint_front_extent"), value_type=float),
                          "footprint_rear_extent": ParameterValue(
                              LaunchConfiguration("footprint_rear_extent"), value_type=float),
                          "footprint_half_width": ParameterValue(
                              LaunchConfiguration("footprint_half_width"), value_type=float),
                          "voxel": 0.2, "max_points": 20000, "max_range": 40.0,
                          "min_height": 0.1, "max_height": SBR_OBSTACLE_MAX_HEIGHT,
                          "ground_plane_radius": 8.0,
                          "ground_clearance": 0.25}],
             output="screen"),
        # beamng_wildos_adapter owns spot1/odom -> spot1/base_link TF.
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             prefix="/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/town_memory_validation_20261014/tracking_contract_candidate/component_tools/controller_debug_prefix.py",
             parameters=[LaunchConfiguration("params_file")],
             remappings=[("cmd_vel", "/wildos/cmd_vel")], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_official",
             parameters=[{"autostart": True,
                          "node_names": ["planner_server", "controller_server"]}],
             output="screen"),
    ])
