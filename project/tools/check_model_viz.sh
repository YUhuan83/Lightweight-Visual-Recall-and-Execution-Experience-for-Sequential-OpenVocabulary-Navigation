#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
echo "== model_visualization messages in 20s =="
timeout 20 ros2 topic echo /spot1/model_visualization --field header.stamp 2>&1 | grep -c "sec:" || true
echo "== model_visualization info =="
ros2 topic info -v /spot1/model_visualization 2>&1 | grep -E 'Type|Publisher count|Subscription count|Node name'
echo "== traversability info =="
ros2 topic info -v /spot1/traversability 2>&1 | grep -E 'Type|Publisher count|Node name'
echo "== object_mask info =="
ros2 topic info -v /spot1/object_mask 2>&1 | grep -E 'Type|Publisher count|Node name'
echo "== recorder model panel check (frame size) =="
ls -la /home/ly/nizhen_viz_capture/frame_00010.png /home/ly/nizhen_viz_capture/frame_00149.png 2>/dev/null
