#!/usr/bin/env bash
# Run ExploRFM traversability and visual-frontier heads on an image.
# Usage: bash run_heads.sh [IMAGE]   (default: ~/fixture_static.png)
set -uo pipefail
IMG="${1:-$HOME/fixture_static.png}"
cd "$HOME/vision_query/vocabulary_query"
rm -rf run_heads
"$HOME/vision-cu128/bin/python" -B explorfm_heads.py \
  --image "$IMG" \
  --checkpoint-root "$HOME/ckpts" \
  --source-root /home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos \
  --output ./run_heads --heatmap relative --smooth || true
echo "=== result.json ==="
cat ./run_heads/result.json 2>/dev/null || echo "no result.json"
