#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/smoothing_boundary_repair_20261018
python3 -m zipfile -e official_wildos_beamng_bundle.zip candidate
python3 - <<'PY'
import hashlib,json
from pathlib import Path
m=json.loads(Path('official_wildos_beamng_bundle.manifest.json').read_text())
for n,h in m['members'].items():assert hashlib.sha256((Path('candidate')/n).read_bytes()).hexdigest()==h,n
print(m['sha256'],len(m['members']))
PY
colcon --log-base build_log build --base-paths candidate/ackermann_smac --build-base build --install-base install --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON >build.log 2>&1
source install/setup.bash
build/wildos_ackermann_smac/test_planner >planner_tests.log 2>&1
build/wildos_ackermann_smac/test_path_contract >path_contract_tests.log 2>&1
build/wildos_ackermann_smac/test_connector >connector_tests.log 2>&1
sha256sum install/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so >library_identity.txt
tail -n 7 planner_tests.log
