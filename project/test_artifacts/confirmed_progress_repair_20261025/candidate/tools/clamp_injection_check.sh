#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

timeout 8 ros2 service call /beamng_bridge/resume beamng_msgs/srv/ChangeSimulationState "{}" >/tmp/clamp_resume.log 2>&1 || true
nohup ros2 launch beamng_wildos_adapter official_graph_stack.launch.py >/tmp/clamp_injection_stack.log 2>&1 &
sleep 12
for _ in 1 2 3 4 5; do
  timeout 5 ros2 topic pub --once /spot1/raw_object_waypoint geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: spot1/odom}, pose: {position: {x: 3713.99, y: 2413.81, z: -418.19}, orientation: {w: 1.0}}}" >/dev/null
  sleep 0.3
done
sleep 3
grep -E 'stable object|obstacle_clamped' /tmp/clamp_injection_stack.log | tail -n 10
bash /home/ly/wildos_server_stop.sh
