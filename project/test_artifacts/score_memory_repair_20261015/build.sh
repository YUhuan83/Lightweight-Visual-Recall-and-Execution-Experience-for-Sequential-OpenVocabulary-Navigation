#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base /home/ly/wildos-console/build_logs build --build-base /home/ly/wildos-console/build --install-base /home/ly/wildos-console/install --merge-install --base-paths /home/ly/wildos-console/bundle/nav2_adapter /home/ly/wildos-console/bundle/official_adapter /home/ly/wildos-console/bundle/wildos_port/repro_check/src/nebula2-wildos/visual_navigation --packages-select wildos_nav2_adapter beamng_wildos_adapter visual_navigation
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
/usr/bin/python3 -m unittest discover -s /home/ly/wildos-console/bundle/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test -p test_scoring_execution_contract.py -v
/usr/bin/python3 -m unittest discover -s /home/ly/wildos-console/bundle/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test -p test_visual_memory.py -v
/usr/bin/python3 -m unittest discover -s /home/ly/wildos-console/bundle/nav2_adapter/test -p test_runtime_contract.py -v
/usr/bin/python3 -c "from pathlib import Path; Path('/home/ly/wildos-console/score_memory_gate_passed.json').write_text('{\"passed\":true}')"
