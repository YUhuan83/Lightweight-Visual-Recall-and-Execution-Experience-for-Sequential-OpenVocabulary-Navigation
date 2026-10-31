#!/usr/bin/env bash
# Linux regression: run the real installer with mocked build tool/setup sources.
# No ROS/BeamNG start, dependency install or real C++ compilation.
set -eo pipefail
FIXTURE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$FIXTURE_ROOT/test_artifacts/console_installer_contract"
FIXTURE_DIR="$(mktemp -d "$FIXTURE_ROOT/test_artifacts/console_installer_contract/fixture.XXXXXX")"
FIXTURE_BIN="$FIXTURE_DIR/build bin"
mkdir -p "$FIXTURE_BIN"
touch "$FIXTURE_DIR/base setup.bash"
ln -s /usr/bin/python3 "$FIXTURE_BIN/python"
cat > "$FIXTURE_BIN/colcon" <<'PY'
#!/usr/bin/python3
import json,sys
from pathlib import Path
args=sys.argv[1:]
prefix=Path(args[args.index('--install-base')+1])
(prefix/'lib').mkdir(parents=True)
for name in ('libwildos_ackermann_smac.so','libwildos_ackermann_rpp.so','libgraphnav_planner_component.so'):
    (prefix/'lib'/name).write_bytes(b'fixture library, not compiled')
(prefix/'setup.bash').touch()
(prefix.parent/'colcon_argv.json').write_text(json.dumps(args))
PY
chmod +x "$FIXTURE_BIN/colcon"
# Skip environment sourcing only inside this isolated fixture shell.
source() { :; }
export -f source
PATH=/usr/bin:/bin WILDOS_BUILD_BIN="$FIXTURE_BIN" bash "$FIXTURE_ROOT/tools/setup_wildos_console.sh" \
  "$FIXTURE_DIR/runtime space" "$FIXTURE_DIR/base setup.bash"
/usr/bin/python3 - "$FIXTURE_DIR/runtime space" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]); conf=json.loads((root/'console.json').read_text())
args=json.loads((root/'colcon_argv.json').read_text())
assert args[args.index('--install-base')+1] == str(root/'install')
packages=args[args.index('--packages-select')+1:args.index('--cmake-args')]
assert len(packages) == 7 and {'graphnav_planner','graphnav_msgs'} <= set(packages)
paths=args[args.index('--base-paths')+1:args.index('--packages-select')]
assert any(path.endswith('/graphnav_msgs') for path in paths)
assert 'libgraphnav_planner_component.so' in conf['library_sha256']
for name,digest in conf['library_sha256'].items():
    assert hashlib.sha256((root/'install/lib'/name).read_bytes()).hexdigest()==digest
assert conf['output_root']==str(root/'results')
print('PASS: clean non-interactive PATH, spaces in build/runtime paths, config/library identity')
PY
# A second invocation must fail before replacing the existing identity.
if PATH=/usr/bin:/bin WILDOS_BUILD_BIN="$FIXTURE_BIN" bash "$FIXTURE_ROOT/tools/setup_wildos_console.sh" \
  "$FIXTURE_DIR/runtime space" "$FIXTURE_DIR/base setup.bash"; then
  echo 'FAIL: installer overwrote an existing runtime' >&2
  exit 1
fi
echo 'PASS: existing runtime refused; fixture evidence retained at' "$FIXTURE_DIR"
