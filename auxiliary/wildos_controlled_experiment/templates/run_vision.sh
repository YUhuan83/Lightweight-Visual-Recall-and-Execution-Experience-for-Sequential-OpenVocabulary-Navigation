#!/usr/bin/env bash
# Start only the selected vision node; control and simulator remain separate.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
python3 "$ROOT/verify_experiment.py" --syntax-only
test -f "$ROOT/install/setup.bash" || { echo 'Run build_ros.sh on the ROS host first'; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-40}
export ROS_LOCALHOST_ONLY=1
CORE="$ROOT/wildos_port/repro_check/src/nebula2-wildos"
export PYTHONPATH="$CORE${PYTHONPATH:+:$PYTHONPATH}"
PYTHON=${WILDOS_VISION_PYTHON:-python3}
"$PYTHON" - "$ROOT" <<'PY'
import hashlib,inspect,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import visual_navigation.wildos.nav as nav
root=Path(sys.argv[1]).resolve()
source=root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'
actual=Path(inspect.getfile(nav)).resolve()
assert actual==source.resolve(), f'Wrong Python overlay: {actual}'
config=Path(get_package_share_directory('visual_navigation'))/'configs/beamng_ablation.yaml'
expected=root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/configs/beamng_ablation.yaml'
assert hashlib.sha256(config.read_bytes()).digest()==hashlib.sha256(expected.read_bytes()).digest(), 'Wrong installed configuration'
print('Verified actual vision source and configuration:',actual,config)
PY
exec "$PYTHON" -m visual_navigation.wildos.nav --config beamng_ablation \
  --do_object_search true --memory_in_process_only
