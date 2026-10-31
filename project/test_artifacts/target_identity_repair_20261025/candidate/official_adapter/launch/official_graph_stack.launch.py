"""Official graphnav mapper/planner/path-follower with BeamNG input adapters.

Does not launch WildOS itself because the official model node must run in the
CUDA vision Python environment. It does not implement alternative planning.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    graph_defaults = dict(
        robot_body_front_extent=-1.0, robot_body_rear_extent=-1.0,
        robot_body_half_width=-1.0, robot_body_height=-1.0,
        robot_footprint_padding=0.0, robot_ground_clearance=-1.0,
        robot_min_turning_radius=-1.0, robot_holonomic=True,
        robot_minimum_clearance=0.30)
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=str(value).lower())
          for name, value in graph_defaults.items()],
        DeclareLaunchArgument("planner_goal_topic", default_value="/spot1/imgnav_waypoint"),
        DeclareLaunchArgument("carrier_model", default_value="ideal_point"),
        DeclareLaunchArgument("vehicle_front_extent", default_value="0.0"),
        DeclareLaunchArgument("state_topic", default_value="/wildos/kinematic_state"),
        Node(
            package="beamng_wildos_adapter",
            executable="beamng_wildos_adapter",
            name="beamng_wildos_adapter",
            parameters=[{"carrier_model": LaunchConfiguration("carrier_model")}],
            remappings=[("/vehicles/ego/sensors/state", LaunchConfiguration("state_topic"))],
            output="screen",
        ),
        Node(
            package="beamng_wildos_adapter",
            executable="goal_standoff",
            name="beamng_vehicle_goal_standoff",
              parameters=[{"carrier_model": LaunchConfiguration("carrier_model"),
                         "vehicle_front_extent": ParameterValue(
                             LaunchConfiguration("vehicle_front_extent"), value_type=float),
                         "standoff": 3.0, "min_update_distance": 1.0,
                         "confirmation_frames": 5, "confirmation_radius": 1.5,
                         "obstacle_margin": 2.0}],
            output="screen",
        ),
        Node(
            package="graphnav_mapper",
            executable="dummy_pose_graph_publisher.py",
            namespace="spot1",
            parameters=[{"frame": "spot1/odom"}],
            output="screen",
        ),
        Node(
            package="graphnav_mapper",
            executable="mapper_node",
            namespace="spot1",
            name="graphnav_mapper",
            parameters=[{"use_occupancy_input": False,
                         **{name: ParameterValue(LaunchConfiguration(name), value_type=type(value))
                            for name, value in graph_defaults.items()}}],
            remappings=[
                ("odometry", "/spot1/odom"),
                ("occupancy", "/spot1/local_map"),
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
            parameters=[{
                "frontier_dist_cost_factor": 2.0,
                "goal_dist_cost_factor": 1.0,
                "frontier_score_factor": 20.0,
                "min_local_frontier_score": 0.4,
                "local_frontier_radius": 10.0,
                "path_smoothness_period": 0.0,
                "trav_class": "default",
                "goal_radius": 6.0,
                "autonomous_exploration_enabled": True,
                "missing_frontier_timeout_seconds": 10.0,
                "use_compact_score_overlay": True,
                "topology_topic": "/spot1/nav_graph",
            }],
            remappings=[
                ("~/nav_graph", "/spot1/scored_nav_graph"),
                ("~/odom", "/spot1/odom"),
                ("~/goal_pose", LaunchConfiguration("planner_goal_topic")),
                ("~/path", "/spot1/graphnav_planner/path"),
            ],
            output="screen",
        ),
        Node(
            package="graphnav_planner",
            executable="path_follower_node",
            namespace="spot1",
            name="graphnav_path_follower",
            parameters=[{"wp_lookahead_dist": 5.0, "path_timeout": 2.0}],
            remappings=[
                ("~/path", "/spot1/graphnav_planner/path"),
                ("~/odom", "/spot1/odom"),
                ("~/goal_pose", "/spot1/goal_pose"),
            ],
            output="screen",
        ),
    ])
