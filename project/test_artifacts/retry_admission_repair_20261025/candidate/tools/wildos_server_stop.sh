#!/usr/bin/env bash
# Emergency stop for the visible server demo. Does not kill BeamNG or bridge.
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

timeout 4 ros2 topic pub --rate 20 /control beamng_msgs/msg/VehicleControl \
  "{steering: 0.0, throttle: 0.0, brake: 1.0, parkingbrake: 0.0, clutch: 0.0, gear: 2}" \
  >/tmp/wildos_emergency_stop.log 2>&1 &
brake_pid=$!
sleep 1
pkill -9 -f cmd_vel_to_control 2>/dev/null
pkill -9 -f path_planner_bridge 2>/dev/null
pkill -9 -f graph_path_bridge 2>/dev/null
pkill -9 -f goal_adapter 2>/dev/null
pkill -9 -f perception_node 2>/dev/null
pkill -9 -f '/install/beamng_wildos_adapter/lib/beamng_wildos_adapter/official_health_check' 2>/dev/null
pkill -9 -f '/install/beamng_wildos_adapter/lib/beamng_wildos_adapter/seed_goal' 2>/dev/null
pkill -9 -f 'visual_navigation.explorfm_triangulation.obj_mask_triangulation' 2>/dev/null
pkill -9 -f 'visual_navigation.wildos.nav' 2>/dev/null
pkill -9 -f '/install/graphnav_planner/lib/graphnav_planner/path_follower_node' 2>/dev/null
pkill -9 -f '/install/graphnav_planner/lib/graphnav_planner/planner_node' 2>/dev/null
pkill -9 -f '/install/graphnav_mapper/lib/graphnav_mapper/mapper_node' 2>/dev/null
pkill -9 -f '/install/graphnav_mapper/lib/graphnav_mapper/dummy_pose_graph_publisher.py' 2>/dev/null
pkill -9 -f '/install/beamng_wildos_adapter/lib/beamng_wildos_adapter/beamng_wildos_adapter' 2>/dev/null
pkill -9 -f '/opt/ros/humble/lib/nav2_controller/controller_server' 2>/dev/null
pkill -9 -f '/opt/ros/humble/lib/nav2_planner/planner_server' 2>/dev/null
pkill -9 -f '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null
timeout 5 ros2 service call /beamng_bridge/pause beamng_msgs/srv/ChangeSimulationState "{}" \
  >>/tmp/wildos_emergency_stop.log 2>&1 || true
wait "$brake_pid" 2>/dev/null || true
echo WILDOS_MOTION_STOPPED
