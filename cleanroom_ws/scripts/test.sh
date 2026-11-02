#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
export PYTHONPATH="$ROOT/src/nav2_adapter:$ROOT/src/official_adapter:$ROOT/src/visual_navigation:${PYTHONPATH}"
cd "$ROOT"
"${TEST_PYTHON:-python3}" -m unittest discover -s tests -p 'test_*.py' -v
