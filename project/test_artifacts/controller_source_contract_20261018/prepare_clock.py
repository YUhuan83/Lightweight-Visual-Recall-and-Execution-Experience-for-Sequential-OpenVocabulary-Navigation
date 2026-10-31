from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'test_artifacts/controller_deadline_clock_20261018';OUT.mkdir(exist_ok=False)
REMOTE='/home/ly/controller_deadline_clock_20261018'
sys.path.insert(0,str(ROOT/'tools'))
from package_official_bundle import write_bundle,bundle_sources
m=write_bundle(ROOT,OUT/'candidate.zip',bundle_sources(ROOT,ROOT/'official_wildos_beamng_bundle.zip'))
before=json.loads((ROOT/'deploy/20261018_controller_source_candidate/official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
changes=[n for n in sorted(m['members']) if m['members'][n]!=before['members'].get(n)]
assert set(changes)=={'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py','nav2_adapter/test/test_controller_source_ros.py'},changes
for name in ('dds_chain.py','server_callbacks.py'):
    text=(Path(__file__).parent/name).read_text(encoding='utf8').replace('/home/ly/controller_source_contract_20261018',REMOTE)
    (OUT/name).write_bytes(text.encode())
script='''set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/controller_source_field_20261018/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd '''+REMOTE+'''
python3 -m zipfile -e candidate.zip candidate
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/candidate/nav2_adapter/config/fastdds_local_udp.xml"
set +e
PYTHONPATH="$PWD/candidate/nav2_adapter/test:$PYTHONPATH" python3 - <<'PY' >before_regression.log 2>&1
import unittest,rclpy
rclpy.init()
try:
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromName('test_controller_source_ros.ControllerSourceTests.test_scheduling_pause_between_source_check_and_issue_brakes_without_exception'))
finally:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
PY
status=$?
set -e
test "$status" = 1
grep -q 'source_lifetime' before_regression.log
colcon --log-base build_log build --base-paths candidate/nav2_adapter --build-base build --install-base install >build.log 2>&1
source install/setup.bash
python3 server_callbacks.py >server_callbacks.log 2>&1
export ROS_DOMAIN_ID=181
python3 dds_chain.py >dds_chain.log 2>&1
tail -2 server_callbacks.log
tail -24 dds_chain.log
'''
(OUT/'run.sh').write_bytes(script.encode())
(OUT/'freeze.json').write_text(json.dumps(dict(candidate=m['sha256'],members=len(m['members']),changes_from_ca8=changes,field_test=False),indent=2)+'\n',encoding='utf8')
print((OUT/'freeze.json').read_text(encoding='utf8'))
