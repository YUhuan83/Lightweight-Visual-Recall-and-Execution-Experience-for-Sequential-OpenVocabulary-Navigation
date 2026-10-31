#!/usr/bin/env bash
# One-time isolated build. Does not launch BeamNG, send a goal or kill processes.
set -eo pipefail
if [ "$#" -ne 2 ]; then
  echo "Usage: bash setup_wildos_console.sh /absolute/new-runtime /absolute/base/install/setup.bash" >&2
  exit 2
fi
CONSOLE_RUNTIME="$1"
CONSOLE_BASE_SETUP="$2"
CONSOLE_BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "$CONSOLE_RUNTIME" in /*) ;; *) echo 'Runtime must be absolute' >&2; exit 2;; esac
test -f "$CONSOLE_BASE_SETUP"
if [ -e "$CONSOLE_RUNTIME/install" ] || [ -e "$CONSOLE_RUNTIME/console.json" ]; then
  echo 'Use a new runtime directory; existing install/config will not be overwritten.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$CONSOLE_BASE_SETUP"
mkdir -p -- "$CONSOLE_RUNTIME"
colcon --log-base "$CONSOLE_RUNTIME/build_logs" build \
  --build-base "$CONSOLE_RUNTIME/build" --install-base "$CONSOLE_RUNTIME/install" \
  --merge-install --base-paths "$CONSOLE_BUNDLE/nav2_adapter" "$CONSOLE_BUNDLE/official_adapter" \
  "$CONSOLE_BUNDLE/ackermann_smac" "$CONSOLE_BUNDLE/ackermann_rpp" \
  "$CONSOLE_BUNDLE/wildos_port/repro_check/src/nebula2-wildos/visual_navigation" \
  --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_smac \
  wildos_ackermann_rpp visual_navigation --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 - "$CONSOLE_RUNTIME" "$CONSOLE_BASE_SETUP" "$CONSOLE_BUNDLE" <<'PY'
import hashlib, json, sys
from pathlib import Path
runtime, base, bundle = map(Path, sys.argv[1:])
config = json.loads((bundle/'config/wildos_console.example.json').read_text())
config['setups'][-1:] = [str(base.resolve()), str(runtime/'install/setup.bash')]
config['python_paths'] = ['/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos']
config['output_root'] = str(runtime/'results')
config['library_sha256'] = {
    name: hashlib.sha256((runtime/'install/lib'/name).read_bytes()).hexdigest()
    for name in ('libwildos_ackermann_smac.so', 'libwildos_ackermann_rpp.so')}
(runtime/'console.json').write_text(json.dumps(config, indent=2)+'\n')
print('Prepared configuration: '+str(runtime/'console.json'))
print('Start: python3 '+str(bundle/'tools/wildos_console.py')+' --config '+str(runtime/'console.json'))
PY
