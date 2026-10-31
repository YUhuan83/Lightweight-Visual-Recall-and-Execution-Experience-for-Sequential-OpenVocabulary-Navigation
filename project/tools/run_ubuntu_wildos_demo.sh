#!/usr/bin/env bash
# Ubuntu local WildOS closed-loop launcher for tools/local_wildos_demo.py.
#
# Shows only the BeamNG American Road view (third-person). The RGB camera and
# LiDAR keep feeding the controller in the background but are never opened as
# extra windows, and per-cycle images/clouds/frontier maps are not written to
# disk (add --save-artifacts to keep them). This maximises smoothness on the
# server while still showing the car move, as the user requested.
#
# For a fully windowless run use: bash run_ubuntu_wildos_demo.sh --headless
set +u

D=/home/ly
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO="$ROOT/local_wildos_demo.py"
[ -f "$DEMO" ] || DEMO="$ROOT/tools/local_wildos_demo.py"
# Main process needs beamngpy; the (separate) vision subprocess needs CUDA torch.
MAIN_PY=${MAIN_PY:-/home/ly/venvs/beamng-039/bin/python}
WORKER_PY=${WORKER_PY:-/home/ly/vision-cu128/bin/python}

QUERY=${1:-house}
EXTRA="${EXTRA:-}"
if [ ! -f "$DEMO" ]; then echo "missing $DEMO"; exit 2; fi

exec "$MAIN_PY" -s -B "$DEMO" \
    --query "$QUERY" \
    --worker-python "$WORKER_PY" \
    --camera-size 960x640 \
    --spawn spawns_sawmill \
    --vision-period 2 \
    --explore-bins 16 \
    --max-cycles 30 \
    --drive-seconds 6 \
    --speed 8 \
    $EXTRA