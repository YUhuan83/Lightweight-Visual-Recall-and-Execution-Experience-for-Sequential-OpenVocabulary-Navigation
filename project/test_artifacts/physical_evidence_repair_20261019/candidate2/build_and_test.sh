#!/usr/bin/env bash
set -eo pipefail
label=${1:?baseline or candidate}
case "$label" in baseline|candidate|candidate2) ;; *) exit 2 ;; esac
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
source /home/ly/goal_release_fence_20261019/install/local_setup.bash
cd /home/ly/physical_evidence_repair_20261019/"$label"
export CMAKE_BUILD_PARALLEL_LEVEL=2 ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1
python_bin=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
$python_bin stage_candidate.py
/home/ly/beamng_ros2_0394_ws/.venv/bin/colcon --log-base colcon_logs build --executor sequential --base-paths candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner --build-base build --install-base install --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo > build.log 2>&1
source install/local_setup.bash
set +e
./build/graphnav_mapper/graphnav_mapper_test_mapper --gtest_color=no --gtest_output=xml:mapper_results.xml > mapper_tests.log 2>&1
mapper_exit=$?
./build/graphnav_planner/test_route_mechanism --gtest_color=no --gtest_output=xml:planner_results.xml > planner_tests.log 2>&1
planner_exit=$?
set -e
$python_bin - "$label" "$mapper_exit" "$planner_exit" <<'PY'
from pathlib import Path
import hashlib,json,sys,xml.etree.ElementTree as ET
label=sys.argv[1];report={'label':label,'vehicle_started':False,'field_acceptance':False,'tests':{}}
for name,status in zip(('mapper','planner'),sys.argv[2:]):
 root=ET.parse(name+'_results.xml').getroot()
 report['tests'][name]={'exit_code':int(status),**{k:int(root.attrib[k]) for k in ('tests','failures','errors')}}
report['passed']=all(v['exit_code']==0 and v['failures']==v['errors']==0 for v in report['tests'].values())
report['libraries']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('install').glob('*/lib/*component.so')}
Path('result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
if label!='baseline' and not report['passed']:raise SystemExit(1)
PY
