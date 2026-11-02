"""One ROS stack, no historical overlays or goal/control handshakes."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = ParameterValue(LaunchConfiguration("port"), value_type=int)
    vehicle_port = ParameterValue(LaunchConfiguration("vehicle_port"), value_type=int)
    navigation = os.environ["WILDOS_CONFIG_DIR"] + "/navigation.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="25252"),
            DeclareLaunchArgument("vehicle_port", default_value="0"),
            Node(
                package="beamng_wildos_adapter",
                executable="beamng_wildos_adapter",
                output="screen",
            ),
            Node(
                package="graphnav_mapper",
                executable="dummy_pose_graph_publisher.py",
                namespace="spot1",
                parameters=[{"frame": "spot1/odom"}],
            ),
            Node(
                package="graphnav_mapper",
                executable="mapper_node",
                namespace="spot1",
                name="graphnav_mapper",
                parameters=[
                    {
                        "use_occupancy_input": False,
                        "robot_holonomic": True,
                        "robot_min_turning_radius": -1.0,
                        "robot_body_front_extent": -1.0,
                        "robot_body_rear_extent": -1.0,
                        "robot_body_half_width": -1.0,
                        "robot_minimum_clearance": 0.30,
                    }
                ],
                remappings=[
                    ("odometry", "/spot1/odom"),
                    ("traversability", "/spot1/local_terrain"),
                    ("pose_graph", "/spot1/pose_graph"),
                ],
                output="screen",
            ),
            Node(
                package="graphnav_planner",
                executable="planner_node",
                namespace="spot1",
                name="graphnav_planner",
                output="screen",
            ),
            Node(
                package="wildos_nav2_adapter",
                executable="cloud_filter",
                parameters=[
                    {
                        "input_topic": "/spot1/ouster/front/points_filtered",
                        "output_topic": "/wildos/points",
                        "clearing_topic": "/wildos/clearing_points",
                        "output_frame": "spot1/base_link",
                        "filter_footprint_returns": True,
                        "footprint_front_extent": 3.4,
                        "footprint_rear_extent": 1.05,
                        "footprint_half_width": 1.05,
                        "voxel": 0.2,
                        "max_points": 20000,
                        "max_range": 40.0,
                        "min_height": 0.1,
                        "max_height": 1.5,
                        "ground_plane_radius": 8.0,
                        "ground_clearance": 0.25,
                        "obstacle_min_height": 0.1,
                        "ground_max_slope": 0.35,
                    }
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[navigation],
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[navigation],
                remappings=[("cmd_vel", "/wildos/cmd_vel")],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["planner_server", "controller_server"],
                    }
                ],
            ),
            Node(
                package="wildos_nav2_adapter",
                executable="graph_path_bridge",
                output="screen",
            ),
            Node(
                package="wildos_nav2_adapter",
                executable="cmd_vel_to_control",
                output="screen",
            ),
            Node(
                package="wildos_nav2_adapter",
                executable="native_vehicle_control",
                parameters=[{"host": host, "port": port, "vehicle_port": vehicle_port}],
                output="screen",
            ),
        ]
    )
