#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
cd /home/ly/vehicle_profile_repair_20261019
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 CMAKE_BUILD_PARALLEL_LEVEL=2
export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter:$PWD/candidate/official_adapter:$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:${PYTHONPATH}
python_bin=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
$python_bin - <<'PY'
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as ET
r=Path.cwd();report=dict(scope='Compiled C++ components and build identities only',tests={},libraries={},complete_navigation_passed=False)
for name in ('mapper','planner'):
    tree=ET.parse(r/(name+'_v2_results.xml')).getroot()
    report['tests'][name]={k:int(tree.attrib[k]) for k in ('tests','failures','errors')}
    assert report['tests'][name]['failures']==report['tests'][name]['errors']==0
for p in (r/'install').glob('*/lib/*component.so'):
    report['libraries'][str(p.relative_to(r))]=hashlib.sha256(p.read_bytes()).hexdigest()
report['passed']=True
(r/'result.json').write_text(json.dumps(report,indent=2)+'\n')
PY
export ROS_DOMAIN_ID=203
$python_bin -m unittest discover -s candidate/nav2_adapter/test -p test_runtime_contract.py -v > runtime_tests.log 2>&1
$python_bin -m unittest discover -s candidate/official_adapter/test -v > official_tests.log 2>&1
$python_bin -m unittest discover -s candidate/nav2_adapter/test -p test_pointcloud.py -v > pointcloud_tests.log 2>&1
$python_bin -m unittest discover -s candidate/nav2_adapter/test -p test_cloud_filter_ros.py -v > cloud_filter_tests.log 2>&1
$python_bin -m unittest discover -s candidate/tools -p test_wildos_console.py -v > console_tests.log 2>&1
bash candidate/tools/test_console_installer.sh > installer_tests.log 2>&1
$python_bin candidate/test_artifacts/goal_release_repair_20261019/run_regression.py --label profile_real_ros --all --real-ros > real_ros.log 2>&1
$python_bin candidate/test_artifacts/recall_contract_repair_20261019/run_tests.py test_execution_prior_contract test_directional_coverage_contract test_visual_memory test_visual_memory_map test_scoring_execution_contract test_target_bearing test_object_memory_integration test_recall_component_names > recall_tests.log 2>&1
export ROS_DOMAIN_ID=195
$python_bin candidate/test_artifacts/goal_release_fence_20261019/probe_joint.py > goal_release_dds.log 2>&1
export ROS_DOMAIN_ID=196
$python_bin probe_recall_dds.py > recall_dds.log 2>&1
export ROS_DOMAIN_ID=198
cmake -S harness -B fixture_build -DMAPPER_PREFIX="$PWD/install/graphnav_mapper" -DCMAKE_BUILD_TYPE=RelWithDebInfo > fixture_build.log 2>&1
cmake --build fixture_build -j2 >> fixture_build.log 2>&1
./fixture_build/fixture_export "$PWD/fixtures_v2" > fixture_export.log 2>&1
$python_bin harness/probe_chain.py > physical_chain.log 2>&1
