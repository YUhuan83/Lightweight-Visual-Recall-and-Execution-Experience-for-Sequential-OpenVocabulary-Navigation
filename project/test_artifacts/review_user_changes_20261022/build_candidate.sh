#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_repair_20261021/reachable_observation_v11/run_full.sh)
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
cd /home/ly/execution_continuity_review_20261022/v1
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter candidate/ackermann_smac --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_smac --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
colcon test --build-base build --install-base install --packages-select wildos_ackermann_smac --event-handlers console_direct+ >smac_tests.log 2>&1
colcon test-result --test-result-base build/wildos_ackermann_smac --verbose >>smac_tests.log 2>&1
export ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH
python real_ros_review.py >real_ros_review.log 2>&1
python - <<'PY'
import hashlib,json
from pathlib import Path
from wildos_nav2_adapter import graph_path_bridge, continuous_path, exploration_cost, motion_path
from beamng_wildos_adapter import goal_standoff
root=Path.cwd()
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())['members']
installed={}
for module in (graph_path_bridge,continuous_path,exploration_cost,motion_path,goal_standoff):
    p=Path(module.__file__).resolve();digest=hashlib.sha256(p.read_bytes()).hexdigest()
    assert p.is_relative_to(root/'install'),str(p)
    assert digest in manifest.values(),str(p)
    installed[module.__name__]={'path':str(p),'sha256':digest}
lib=root/'install/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so'
config=json.loads((root/'full/config.json').read_text())
config['library_sha256'][lib.name]=hashlib.sha256(lib.read_bytes()).hexdigest()
(root/'full/config.json').write_text(json.dumps(config,indent=2))
(root/'installed_identity.json').write_text(json.dumps(installed,indent=2))
PY
