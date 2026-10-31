#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec "${RURAL_PYTHON:-$HERE/.venv/bin/python}" "$HERE/run.py" "$@"
