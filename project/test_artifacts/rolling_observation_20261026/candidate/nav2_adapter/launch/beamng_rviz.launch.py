"""Visualize the running BeamNG bridge; this launch sends no driving commands."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config=os.path.join(get_package_share_directory('wildos_nav2_adapter'),'config','beamng_navigation.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config',default_value=config),
        Node(package='wildos_nav2_adapter',executable='visualization_bridge',output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',LaunchConfiguration('rviz_config')],output='screen'),
    ])
