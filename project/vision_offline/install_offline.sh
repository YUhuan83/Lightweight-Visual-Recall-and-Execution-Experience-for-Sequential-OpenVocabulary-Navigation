#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV="${1:-$HOME/vision-cu128}"
PYTHON="${PYTHON:-python3.10}"
export PYTHONDONTWRITEBYTECODE=1 PIP_NO_INDEX=1 PIP_DISABLE_PIP_VERSION_CHECK=1
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1
unset PYTHONPATH PYTHONHOME
"$PYTHON" - "$VENV" <<'PY'
import os, pathlib, platform, shutil, sys
p = pathlib.Path(sys.argv[1]).expanduser().resolve()
assert os.geteuid() != 0, 'Run as an ordinary user, not root'
assert sys.version_info[:2] == (3, 10), 'Requires Python 3.10'
assert platform.system() == 'Linux' and platform.machine() == 'x86_64'
assert p.is_relative_to(pathlib.Path.home().resolve()), 'Venv must be inside your home'
assert not p.exists(), 'Refusing to modify an existing environment'
assert not pathlib.Path(sys.argv[1]).is_symlink(), 'Refusing a symlink destination'
assert p.parent.is_dir(), 'Venv parent must already exist'
assert shutil.disk_usage(p.parent).free >= 15 * 1024**3, 'Need at least 15 GiB free'
PY
# ensurepip uses the Python distribution's bundled pip, without network access.
"$PYTHON" -m venv "$VENV"
"$VENV/bin/python" -B "$HERE/kit.py" verify
"$VENV/bin/python" -m pip --isolated install --no-index --no-cache-dir \
  --find-links "$HERE/wheelhouse" --only-binary=:all: --require-hashes \
  -r "$HERE/requirements.lock"
"$VENV/bin/python" -m pip check
"$VENV/bin/python" -B "$HERE/verify_runtime.py"
printf 'Environment ready: %s\n' "$VENV"
