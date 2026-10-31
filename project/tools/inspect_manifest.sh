#!/usr/bin/env bash
set +u
echo "=== procs ==="
for p in 'BeamNG.tech.x64 -nosteam' 'visual_navigation.wildos.nav' 'rviz2 -d' 'watchdog_nizhen.sh' 'mjpeg_stream.py'; do
  if pgrep -f "$p" >/dev/null 2>&1; then echo "ALIVE  $p"; else echo "DEAD   $p"; fi
done
echo "=== manifest top keys ==="
python3 - <<'PY'
import json
m=json.load(open('/home/ly/nizhen_countryside_deploy/standalone_rural/generated/build_manifest.json'))
print('TOP KEYS', list(m.keys()))
for k,v in m.items():
    if isinstance(v,dict):
        print(k, '->', list(v.keys())[:15])
    elif isinstance(v,list):
        print(k, '-> list len', len(v))
    else:
        print(k, '=', v)
PY
echo "=== map items with position (houses/roads) ==="
python3 - <<'PY'
import json,glob,os
base='/home/ly/nizhen_countryside_deploy/standalone_rural/generated'
# try to find the extracted map items json inside the zip listing or generated dir
for p in glob.glob(base+'/**/*.json', recursive=True):
    print('JSON', p)
PY
