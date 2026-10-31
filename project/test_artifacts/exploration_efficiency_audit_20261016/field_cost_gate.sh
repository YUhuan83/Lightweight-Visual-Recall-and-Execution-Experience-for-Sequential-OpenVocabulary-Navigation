#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
export PYTHONPATH=/home/ly/wildos-console/bundle/tools:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
from pathlib import Path
import hashlib,json,shutil
from ament_index_python.packages import get_package_prefix
from graphnav_msgs.msg import ExplorationChoice
import wildos_console as m
import wildos_nav2_adapter.exploration_cost as cost
r=Path('/home/ly/frontier_cost_repair_20261016');cfg=json.loads((r/'field/sensor_full_console.json').read_text())
identity=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
assert identity['sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
for n,h in identity['members'].items():assert hashlib.sha256((Path('/home/ly/wildos-console/bundle')/n).read_bytes()).hexdigest()==h,n
assert get_package_prefix('graphnav_planner')==str(r/'install')
assert get_package_prefix('graphnav_msgs')==str(r/'install')
assert Path(cost.__file__).is_relative_to(r/'release_install')
assert shutil.disk_usage(r).free>20*1024**3
verified=m.verify_sources(cfg)
(r/'runtime_gate.json').write_text(json.dumps(dict(source=identity['sha256'],verified=verified,
    graphnav_prefix=get_package_prefix('graphnav_planner'),messages_prefix=get_package_prefix('graphnav_msgs'),
    cost_module=cost.__file__,choice_fields=ExplorationChoice.get_fields_and_field_types()),indent=2))
print('Frozen source, installed code, generated interfaces and C++ library gate passed')
PY
