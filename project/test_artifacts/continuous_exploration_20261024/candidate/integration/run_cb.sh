#!/usr/bin/env bash
# Run multi-frame e2e for the fixture with an explicit large-target policy.
# Usage: bash run_cb.sh [RUN_ID]
set -euo pipefail
RUN="${1:-a221141e}"
cd ~/vision_query
rm -rf "$HOME/vision_query/vocabulary_query/integration_${RUN}_frame_"*
~/vision-cu128/bin/python -m integration.run_e2e \
  --frames-dir "$HOME/capture_out/$RUN/frames" \
  --query "black white checkerboard" \
  --source-root /home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos \
  --checkpoint-root /home/ly/ckpts \
  --query-script /home/ly/vision_query/vocabulary_query/query.py \
  --query-python /home/ly/vision-cu128/bin/python \
  --accept-calibration "fixture nominal K (FOV 60deg); api_fit |f| within 0.5% of nominal 554.256; static visual reprojection < 0.4px" \
  --max-extent-m 8 --max-depth-span-m 3.5 \
  --max-ring-same-depth-fraction 0.99 --alpha-silhouette 0.01 \
  --ttl-ns 30000000000 \
  --output "$HOME/e2e_out"
python3 -c "
import json
r = json.load(open('/home/ly/e2e_out/integration_report.json'))
print('loc:', json.dumps(r['stages']['localization'], indent=2))
for f in r['frames']:
    h = f['handoff']; o = h.get('observation')
    print(f['frame'], h.get('state'), h.get('confirmation_count'), h.get('reason'))
    if o: print('   rejected=', o.get('rejected'), 'candidates=', o.get('candidates'))
"
