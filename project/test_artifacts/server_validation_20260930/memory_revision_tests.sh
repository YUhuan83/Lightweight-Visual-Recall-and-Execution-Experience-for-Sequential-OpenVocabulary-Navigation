#!/usr/bin/env bash
set -eo pipefail
stage=/home/ly/memory_revision_20260930_stage
mkdir -p "$stage"
unzip -oq /home/ly/memory_revision_20260930.zip -d "$stage"
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=40 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$stage/nav2_adapter:$stage/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
python3 -m pytest -q "$stage/nav2_adapter/test/test_graph_path_bridge_routing.py" "$stage/nav2_adapter/test/test_graph_route.py" "$stage/nav2_adapter/test/test_memory_candidates.py" "$stage/nav2_adapter/test/test_pointcloud.py" "$stage/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_viewpoint_memory.py" "$stage/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_viewpoint_callbacks.py"
