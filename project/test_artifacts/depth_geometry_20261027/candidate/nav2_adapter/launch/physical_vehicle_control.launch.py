"""Town SBR native physics stack; attach to an already loaded sensor rig.

Vision, triangulation, sensor bridge and RViz are separate runtime owners.
Restore and verify the original JBeam configuration before starting this stack.
Both command gates start closed; server_test_preflight must pass before arming.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from wildos_nav2_adapter.vehicle_geometry import (
    SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH, SBR_WHEELBASE)
from wildos_nav2_adapter.runtime_contract import graph_geometry_profile


def generate_launch_description():
    nav = get_package_share_directory('wildos_nav2_adapter')
    official = get_package_share_directory('beamng_wildos_adapter')
    with open(os.path.join(nav, 'config', 'nav2_ackermann_control.yaml'), encoding='utf8') as stream:
        nav_config = yaml.safe_load(stream)
        graph_profile = graph_geometry_profile(nav_config)
        planner_radius = float(nav_config['planner_server']['ros__parameters']['CarLike']['minimum_turning_radius'])
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='25252'),
        DeclareLaunchArgument('vehicle_port', default_value='0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(official, 'launch', 'official_graph_stack.launch.py')),
            launch_arguments={
                'planner_goal_topic': '/spot1/graphnav_destination',
                'carrier_model': 'ackermann',
                'vehicle_front_extent': str(SBR_FRONT_EXTENT),
                'observation_minimum_turning_radius': str(max(planner_radius,graph_profile['robot_min_turning_radius'])),
                'state_topic': '/vehicles/ego/sensors/state',
                **{name: str(value).lower() for name, value in graph_profile.items()}}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nav, 'launch', 'nav2_official_control.launch.py')),
            launch_arguments={
                'params_file': os.path.join(nav, 'config', 'nav2_ackermann_control.yaml'),
                'footprint_front_extent': str(SBR_FRONT_EXTENT),
                'footprint_rear_extent': str(SBR_REAR_EXTENT),
                'footprint_half_width': str(SBR_HALF_WIDTH),
                'filter_footprint_returns': 'true'}.items()),
        Node(package='wildos_nav2_adapter', executable='continuous_graph_path_bridge',
             name='wildos_graph_path_bridge', parameters=[{
                 'follow_path': False, 'carrier_model': 'ackermann', 'require_source_control': True,
                 'planner_id': 'CarLike', 'minimum_turning_radius': 8.0,
                 'continuous_path_execution': True,
                 'planner_minimum_turning_radius': planner_radius}],
             output='screen'),
        Node(package='wildos_nav2_adapter', executable='cmd_vel_to_control',
             name='wildos_cmd_vel_to_control', parameters=[{
                 'enable': False, 'allow_reverse': True, 'use_sim_time': False,
                 'cmd_vel_topic': '/wildos/cmd_vel_stamped',
                 'require_stamped_command': True, 'cmd_vel_frame': 'spot1/nav_base_link',
                 'state_topic': '/vehicles/ego/sensors/state',
                 'control_topic': '/wildos/native_control_command',
                 'control_protocol': 'wildos_source_control_v1',
                 'minimum_turning_radius': 8.0, 'wheelbase': SBR_WHEELBASE,
                 'gear_change_confirmation_time': 0.3,
                 'max_speed': 0.0, 'overspeed_limit': 0.0}], output='screen'),
        Node(package='wildos_nav2_adapter', executable='native_vehicle_control',
             name='wildos_native_vehicle_control', parameters=[{
                 'control_topic': '/wildos/native_control_command',
                 'control_protocol': 'wildos_source_control_v1',
                 'host': LaunchConfiguration('host'),
                 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                 'vehicle_port': ParameterValue(
                     LaunchConfiguration('vehicle_port'), value_type=int)}], output='screen'),
    ])
