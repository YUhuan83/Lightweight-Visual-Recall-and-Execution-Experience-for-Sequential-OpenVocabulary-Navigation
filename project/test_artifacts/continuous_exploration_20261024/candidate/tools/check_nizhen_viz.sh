#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
for t in /spot1/model_visualization /spot1/realsense/front/color/image_raw /spot1/ouster/front/points_filtered /spot1/navgraph_viz /spot1/nav_graph_viz; do
  echo "== $t =="
  timeout 6 ros2 topic hz "$t" 2>&1 | head -n 2
done
echo "== recorder log =="
tail -n 6 /tmp/official_recorder.log 2>/dev/null
echo "== captures =="
ls -la /home/ly/nizhen_viz_capture 2>/dev/null | tail -n 8
echo "== rviz log tail =="
tail -n 12 /tmp/official_rviz.log 2>/dev/null
