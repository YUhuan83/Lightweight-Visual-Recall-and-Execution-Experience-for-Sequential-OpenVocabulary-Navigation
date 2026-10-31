#!/usr/bin/env bash
set -uo pipefail
export DISPLAY=:6
export XAUTHORITY=/run/user/1011/gdm/Xauthority
D=/home/ly
VQ=$D/vision_query
SRC=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
DUR="${DUR:-90}"

echo "== stopping any running BeamNG =="
pkill -f 'BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64' 2>/dev/null
sleep 3

cd "$VQ"
echo "== continuous live stream for ${DUR}s (no freeze, car held) =="
"$D/venvs/beamng-039/bin/python" -B integration/live_stream.py \
  --home "$D/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0" \
  --user "$D/.local/share/BeamNG/BeamNG.tech" \
  --replace-current --seconds "$DUR" --gfx vk \
  --checkpoint-root "$D/ckpts" --source-root "$SRC" \
  --heads-python "$D/vision-cu128/bin/python"
echo "DONE"
