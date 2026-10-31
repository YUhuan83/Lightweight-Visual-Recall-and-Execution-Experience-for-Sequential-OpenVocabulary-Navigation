#!/usr/bin/env bash
set -eo pipefail
R=/home/ly/frontier_cost_repair_20261016
source /home/ly/exploration_efficiency_audit_20261016/env.sh
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python - <<'PY'
import hashlib,json,zipfile
from pathlib import Path
r=Path('/home/ly/frontier_cost_repair_20261016')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
a=r/'official_wildos_beamng_bundle.zip';assert hashlib.sha256(a.read_bytes()).hexdigest()==m['sha256']
b=r/'candidate';b.mkdir(exist_ok=False)
with zipfile.ZipFile(a) as z:
 assert set(z.namelist())==set(m['members'])
 for n,h in m['members'].items():
  p=b/n;assert p.resolve().is_relative_to(b.resolve());v=z.read(n)
  assert hashlib.sha256(v).hexdigest()==h;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(v)
old=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
diff=[n for n,h in m['members'].items() if old['members'].get(n)!=h]
(r/'delta.json').write_text(json.dumps(dict(source=m['sha256'],base=old['sha256'],changed=diff),indent=2))
PY
colcon --log-base "$R/build_logs" build --build-base "$R/build" --install-base "$R/install" --merge-install --base-paths "$R/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_msgs" "$R/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner" "$R/candidate/nav2_adapter" --packages-select graphnav_msgs graphnav_planner wildos_nav2_adapter --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source "$R/install/setup.bash"
export PYTHONPATH="$R/candidate/nav2_adapter/test:/home/ly/exploration_owner_repair_20261016:$PYTHONPATH"
cd "$R/build/graphnav_planner"
ctest --output-on-failure -R '^test_route_mechanism$'
python -m unittest test_exploration_cost_ros.ExplorationCostTests -v
python "$R/test_frontier_choice_dds.py"
