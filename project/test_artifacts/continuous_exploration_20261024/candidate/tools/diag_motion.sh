#!/usr/bin/env bash
set +u
echo "=== port 25252 ==="
ss -ltn 2>/dev/null | grep 25252 || echo NO_PORT
echo "=== bridge log ==="
tail -n 15 /tmp/official_bridge.log 2>/dev/null || echo no_bridge_log
echo "=== beamng log tail ==="
tail -n 12 /tmp/official_beamng.log 2>/dev/null || echo no_beamng_log
echo "=== crash reports ==="
ls -lt /home/ly/nizhen-user2/current/temp/crashReports/*.dmp 2>/dev/null | head -n 3 || echo none
echo "=== all ros procs ==="
pgrep -af 'beamng_bridge|graphnav|visual_navigation|wildos' | grep -v pgrep | head -n 10 || echo none
