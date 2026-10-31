#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
audit_root=/home/ly/frontier_resolution_repair_20261016
python - <<'PY'
from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/frontier_resolution_repair_20261016')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
d=json.loads((r/'delta.json').read_text())
old=json.loads(Path('/home/ly/exploration_scope_repair_20261016/official_wildos_beamng_bundle.manifest.json').read_text())
assert m['sha256']==d['candidate'] and old['sha256']==d['base']
assert sorted(n for n,h in m['members'].items() if old['members'].get(n)!=h)==d['changed']
assert hashlib.sha256((r/'official_wildos_beamng_bundle.zip').read_bytes()).hexdigest()==m['sha256']
out=r/'candidate';out.mkdir(exist_ok=False)
with zipfile.ZipFile(r/'official_wildos_beamng_bundle.zip') as z:
 assert set(z.namelist())==set(m['members']) and z.testzip() is None
 for n,h in m['members'].items():
  b=z.read(n);assert hashlib.sha256(b).hexdigest()==h
  p=out/n;assert p.resolve().is_relative_to(out.resolve());p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
PY
colcon --log-base "$audit_root/build_logs" build --build-base "$audit_root/build" --install-base "$audit_root/install" --merge-install --base-paths "$audit_root/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner" --packages-select graphnav_planner --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON >"$audit_root/build_validation.log" 2>&1
source "$audit_root/install/setup.bash"
export PYTHONPATH="$audit_root/candidate/nav2_adapter/test:$audit_root:$PYTHONPATH"
ctest --test-dir "$audit_root/build/graphnav_planner" --output-on-failure -R '^test_route_mechanism$' >"$audit_root/cpp_after.log" 2>&1
python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests test_exploration_cost_ros -v >"$audit_root/ros_after.log" 2>&1
python /home/ly/frontier_cost_repair_20261016/test_frontier_choice_dds.py >"$audit_root/dds_existing.log" 2>&1
python /home/ly/exploration_scope_repair_20261016/test_scope_dds.py >"$audit_root/dds_scope.log" 2>&1
python "$audit_root/test_resolution_dds.py" >"$audit_root/dds_resolution.log" 2>&1
python - <<'PY'
from pathlib import Path
import hashlib,json
r=Path('/home/ly/frontier_resolution_repair_20261016')
p=Path('/home/ly/wildos-console');old=json.loads((p/'source_identity.json').read_text())
assert old['sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
for n,h in old['members'].items():assert hashlib.sha256((p/'bundle'/n).read_bytes()).hexdigest()==h,n
lib=r/'install/lib/libgraphnav_planner_component.so'
result=dict(candidate=json.loads((r/'delta.json').read_text())['candidate'],production=old['sha256'],production_unchanged=True,library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest(),vehicle_run=False)
(r/'validation_identity.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
PY
