#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
audit_root=/home/ly/exploration_scope_repair_20261016
python - <<'PY'
from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/exploration_scope_repair_20261016')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
d=json.loads((r/'delta.json').read_text())
old=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
assert m['sha256']==d['candidate'] and old['sha256']==d['base']
assert hashlib.sha256((r/'official_wildos_beamng_bundle.zip').read_bytes()).hexdigest()==m['sha256']
assert sorted(n for n,h in m['members'].items() if old['members'].get(n)!=h)==sorted(d['changed'])
out=r/'candidate';out.mkdir(exist_ok=False)
with zipfile.ZipFile(r/'official_wildos_beamng_bundle.zip') as z:
 assert set(z.namelist())==set(m['members']) and z.testzip() is None
 for name,h in m['members'].items():
  b=z.read(name);assert hashlib.sha256(b).hexdigest()==h
  p=out/name;assert p.resolve().is_relative_to(out.resolve());p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
PY
colcon --log-base "$audit_root/build_logs" build --build-base "$audit_root/build" --install-base "$audit_root/install" --merge-install --base-paths "$audit_root/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner" "$audit_root/candidate/nav2_adapter" --packages-select graphnav_planner wildos_nav2_adapter --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON >"$audit_root/build_validation.log" 2>&1
source "$audit_root/install/setup.bash"
export PYTHONPATH="$audit_root/candidate/nav2_adapter/test:/home/ly/exploration_owner_repair_20261016:$PYTHONPATH"
ctest --test-dir "$audit_root/build/graphnav_planner" --output-on-failure -R '^test_route_mechanism$' >"$audit_root/cpp_after.log" 2>&1
python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests test_exploration_cost_ros -v >"$audit_root/ros_after.log" 2>&1
python /home/ly/frontier_cost_repair_20261016/test_frontier_choice_dds.py >"$audit_root/dds_after.log" 2>&1
python - <<'PY'
from pathlib import Path
import hashlib,json
import wildos_nav2_adapter.graph_path_bridge as bridge
import wildos_nav2_adapter.execution_recovery as recovery
r=Path('/home/ly/exploration_scope_repair_20261016');m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
loaded={}
for module in (bridge,recovery):
 p=Path(module.__file__);h=hashlib.sha256(p.read_bytes()).hexdigest()
 assert p.is_relative_to(r/'install') and h==m['members']['nav2_adapter/wildos_nav2_adapter/'+p.name]
 loaded[str(p)]=h
production=Path('/home/ly/wildos-console');old=json.loads((production/'source_identity.json').read_text())
assert old['sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
for name,h in old['members'].items():assert hashlib.sha256((production/'bundle'/name).read_bytes()).hexdigest()==h,name
lib=r/'install/lib/libgraphnav_planner_component.so'
(r/'validation_identity.json').write_text(json.dumps(dict(candidate=m['sha256'],production=old['sha256'],production_unchanged=True,installed=loaded,library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest(),vehicle_run=False),indent=2))
print('Candidate components passed; production unchanged')
PY
