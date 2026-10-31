#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
export PYTHONPATH=/home/ly/wildos-console/bundle/tools:/home/ly/wildos-console/bundle/nav2_adapter/test:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
import json,sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import wildos_console as m
r=Path('/home/ly/exploration_owner_field_20261016');cfg=json.loads((r/'sensor_full_console.json').read_text());h=m.verify_sources(cfg)
assert get_package_prefix('graphnav_planner')=='/home/ly/exploration_owner_repair_20261016/final_install'
(r/'runtime_gate.json').write_text(json.dumps(dict(verified=h,graphnav_prefix=get_package_prefix('graphnav_planner')),indent=2));print('Runtime source/launch/config/library identity gate passed',flush=True)
PY
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests -v
