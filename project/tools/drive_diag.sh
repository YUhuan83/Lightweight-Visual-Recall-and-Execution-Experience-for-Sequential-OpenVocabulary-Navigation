#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
echo "== /wildos/cmd_vel =="
timeout 6 ros2 topic hz /wildos/cmd_vel 2>&1 | grep -m1 'average rate' || echo "no cmd_vel"
echo "== /control =="
timeout 6 ros2 topic hz /control 2>&1 | grep -m1 'average rate' || echo "no control"
echo "== cmd_vel_to_control log =="
tail -n 12 /tmp/official_control.log 2>/dev/null
echo "== nav2 errors =="
grep -n -iE 'error|fail|invalid|died|timed out|no valid' /tmp/official_nav2.log 2>/dev/null | tail -n 15
echo "== local plan log =="
tail -n 6 /tmp/official_local_plan.log 2>/dev/null
echo "== tf frames =="
timeout 6 ros2 topic echo /tf --once 2>/dev/null | grep -E 'frame_id|child_frame_id' | head -n 12
