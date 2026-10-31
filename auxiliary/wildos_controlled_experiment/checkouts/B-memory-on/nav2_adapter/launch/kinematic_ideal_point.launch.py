"""Start the fixed-node ideal sensor carrier with no steering-radius model."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wildos_nav2_adapter",
            executable="kinematic_vehicle_control",
            name="wildos_kinematic_vehicle_control",
            parameters=[{
                "enable": False,
                "carrier_model": "ideal_point",
                "minimum_turning_radius": 0.0,
                "max_angular_rate": 0.8,
            }],
            output="screen",
        ),
    ])
