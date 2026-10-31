#!/usr/bin/env bash
set -eo pipefail
stage=/home/ly/navigation_validation_20260930_stage
mkdir -p "$stage"
echo 'd52af186a0992ab326eab9b982af23fa9a09c208f0ccfbff9ce0f200af6c456a  /home/ly/navigation_validation_20260930.zip' | sha256sum -c -
unzip -oq /home/ly/navigation_validation_20260930.zip -d "$stage"
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=40 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export PYTHONPATH="$stage/nav2_adapter:$stage/official_adapter:$PYTHONPATH"
python3 -m pytest -q "$stage/nav2_adapter/test/test_graph_route.py" "$stage/nav2_adapter/test/test_graph_path_bridge_routing.py" "$stage/nav2_adapter/test/test_pointcloud.py"
