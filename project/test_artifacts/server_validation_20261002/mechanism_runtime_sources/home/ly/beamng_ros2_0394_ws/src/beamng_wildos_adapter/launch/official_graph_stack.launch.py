"""Official graphnav mapper/planner/path-follower with BeamNG input adapters.

Does not launch WildOS itself because the official model node must run in the
CUDA vision Python environment. It does not implement alternative planning.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planner_goal_topic", default_value="/spot1/imgnav_waypoint"),
        Node(
            package="beamng_wildos_adapter",
            executable="beamng_wildos_adapter",
            name="beamng_wildos_adapter",
            output="screen",
        ),
        Node(
            package="beamng_wildos_adapter",
            executable="goal_standoff",
            name="beamng_vehicle_goal_standoff",
            parameters=[{"standoff": 5.0, "min_update_distance": 1.0,
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
            remappings=[
                ("odometry", "/spot1/odom"),
                ("occupancy", "/spot1/local_map"),
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
