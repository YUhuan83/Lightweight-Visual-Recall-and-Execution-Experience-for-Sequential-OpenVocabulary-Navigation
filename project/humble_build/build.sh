#!/usr/bin/env bash
# Explicit build and optional upstream fetch. Never launches nodes.
set -eo pipefail
if [[ $# -lt 2 || $# -gt 3 || "$2" != --no-server-precheck || ( $# -eq 3 && "$3" != --fetch-graaf ) ]]; then
  echo 'Usage: bash build.sh NEW_WORKSPACE --no-server-precheck [--fetch-graaf]' >&2
  exit 1
fi
WS="$1"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Missing ROS 2 Humble; no build performed' >&2; exit 2; }
source /opt/ros/humble/setup.bash
python3 -B - "$WS" "$HERE" <<'PY'
import json, sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from preflight import snapshot
p = Path(sys.argv[1]).resolve(strict=True)
assert Path.home().resolve() in p.parents, 'Workspace must be under home'
r = json.loads((p / 'prepared.json').read_text())
assert r['destination'] == str(p), 'Workspace identity mismatch'
assert snapshot(p / 'src/nebula2-wildos') == r['staged_snapshot'], 'Staged source changed; re-audit'
PY
WS="$(cd -- "$WS" && pwd)"
mkdir -p "$WS/reports"
bash "$HERE/run.sh" --preflight --source "$WS/src/nebula2-wildos" --no-server-precheck > "$WS/reports/build-preflight.json"
GRAAF="$WS/vendor/graaf"
if [[ ${3:-} == --fetch-graaf ]]; then
  [[ ! -e "$GRAAF" ]] || { echo 'Graaf destination exists; refusing fetch overwrite' >&2; exit 1; }
  mkdir -p "$WS/vendor"
  git clone --depth 1 --branch v1.1.1 https://github.com/bobluppes/graaf.git "$GRAAF"
fi
[[ -f "$GRAAF/include/graaflib/graph.h" ]] || { echo 'Missing Graaf source; explicitly allow --fetch-graaf or supply verified v1.1.1 clone' >&2; exit 2; }
[[ "$(git -C "$GRAAF" remote get-url origin)" == https://github.com/bobluppes/graaf.git ]]
[[ "$(git -C "$GRAAF" rev-parse HEAD)" == "$(git -C "$GRAAF" rev-parse 'v1.1.1^{commit}')" ]]
[[ -z "$(git -C "$GRAAF" status --porcelain --untracked-files=all)" ]]
git -C "$GRAAF" rev-parse HEAD > "$WS/reports/graaf-commit.txt"
# -S/-B and every colcon output path stay in the dedicated home workspace.
cd -- "$WS"
export CMAKE_BUILD_PARALLEL_LEVEL=2
COMMON=(--base-paths "$WS/src" --build-base "$WS/build" --install-base "$WS/install" --executor sequential --event-handlers console_direct+)
CMAKE=(-DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "-DFETCHCONTENT_SOURCE_DIR_GRAAF_SRC=$GRAAF" -DFETCHCONTENT_FULLY_DISCONNECTED=ON)
colcon --log-base "$WS/log/interfaces" build "${COMMON[@]}" --packages-up-to pose_graph_msgs graphnav_msgs object_search_msgs gps_visualization --cmake-args "${CMAKE[@]}" 2>&1 | tee "$WS/reports/01-interfaces.log"
source "$WS/install/local_setup.bash"
colcon --log-base "$WS/log/graph" build "${COMMON[@]}" --packages-up-to graphnav_mapper graphnav_planner --cmake-args "${CMAKE[@]}" 2>&1 | tee "$WS/reports/02-graph.log"
colcon --log-base "$WS/log/python" build "${COMMON[@]}" --packages-up-to triangulation3d visual_navigation --cmake-args "${CMAKE[@]}" 2>&1 | tee "$WS/reports/03-python.log"
echo 'Compilation completed. Runtime imports, tests, models and navigation are NOT validated.'
