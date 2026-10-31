from pathlib import Path
import hashlib,json,sys,zipfile
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
sys.path.insert(0,str(root/'tools'))
from package_official_bundle import write_bundle,bundle_sources
manifest=write_bundle(root,out/'candidate.zip',bundle_sources(root,root/'official_wildos_beamng_bundle.zip'))
remote='/home/ly/controller_source_contract_20261018'
(out/'server_callbacks.py').write_text('''from pathlib import Path
import json,hashlib,sys,unittest,rclpy
root=Path("'''+remote+'''")
manifest=json.loads((root/'candidate.manifest.json').read_text())
for name,want in manifest['members'].items():
    assert hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()==want,name
sys.path[:0]=[str(root/'candidate/nav2_adapter/test'),str(root/'candidate/tools')]
import wildos_nav2_adapter.cmd_vel_to_control as implementation
assert str(Path(implementation.__file__).resolve()).startswith(str(root/'install'))
assert hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest()==manifest['members']['nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py']
rclpy.init()
try:
    suite=unittest.defaultTestLoader.loadTestsFromNames([
        'test_controller_source_ros.ControllerSourceTests',
        'test_source_control_ros.SourceControlCallbackTests',
        'test_execution_transport_health_ros.ExecutionTransportHealthTests',
        'test_tracking_handoff_ros.TrackingHandoffTests',
        'test_motion_intent_handoff_ros.MotionIntentHandoffTests',
        'test_runtime_contract.RuntimeContractTests',
        'test_source_control_preflight.SourceEndpointTests'])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    print(json.dumps(dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        passed=result.wasSuccessful(),source=implementation.__file__,simulation=False)))
finally:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
''',encoding='utf8')
(out/'run_component.sh').write_bytes(('''set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/motion_handoff_field_20261018/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
R='''+remote+'''
export ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd "$R"
python3 -m zipfile -e candidate.zip candidate
export FASTRTPS_DEFAULT_PROFILES_FILE="$R/candidate/nav2_adapter/config/fastdds_local_udp.xml"
python3 - <<'PY'
from pathlib import Path
import shutil
r=Path('.')
assert not (r/'baseline').exists()
shutil.copytree(r/'candidate/ackermann_rpp',r/'baseline/ackermann_rpp')
shutil.copy2(r/'before_controller.cpp',r/'baseline/ackermann_rpp/src/controller.cpp')
PY
colcon --log-base baseline_log build --base-paths baseline/ackermann_rpp --build-base baseline_build --install-base baseline_install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON >baseline_build.log 2>&1
source baseline_install/setup.bash
set +e
baseline_build/wildos_ackermann_rpp/test_controller --gtest_filter=ActualPlugin.PhysicalCommandPublishesGenerationTimeRobotFrameAndExactTwist >baseline_reproduction.log 2>&1
BASELINE_STATUS=$?
set -e
test "$BASELINE_STATUS" = 1
grep -q 'FAILED' baseline_reproduction.log
colcon --log-base candidate_log build --base-paths candidate/ackermann_rpp candidate/nav2_adapter --build-base build --install-base install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON >build.log 2>&1
source install/setup.bash
colcon --log-base test_log test --base-paths candidate/ackermann_rpp --build-base build --install-base install --event-handlers console_direct+ >cpp_tests.log 2>&1
colcon test-result --test-result-base build --verbose >cpp_summary.log
python3 server_callbacks.py >server_callbacks.log 2>&1
python3 - <<'PY'
from pathlib import Path
import json,hashlib
r=Path('.')
lib=r/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
summary=dict(candidate=json.loads((r/'candidate.manifest.json').read_text())['sha256'],
    cpp_summary=(r/'cpp_summary.log').read_text(),
    rpp_library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest(),field_test=False)
(r/'component_summary.json').write_text(json.dumps(summary,indent=2)+'\\n')
print(json.dumps(summary,indent=2))
PY
''').encode())
print(json.dumps(dict(members=len(manifest['members']),candidate=manifest['sha256']),indent=2))
