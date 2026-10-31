#!/usr/bin/env bash
# Run WildOS-style coarse goal localization on captured frames + query masks.
# Usage: bash run_objectsearch.sh [RUN_ID] [extra flags...]
#   e.g. --hypothesis-xyz 0 -10 2.35 --distribution gaussian_hyp --hypothesis-std 1.0
set -uo pipefail
RUN="${1:-a221141e}"
shift || true
cd "$HOME/vision_query"
"$HOME/vision-cu128/bin/python" -m object_search.run_object_search \
  --frames-dir "$HOME/capture_out/$RUN/frames" \
  --vocab-dir "$HOME/vision_query/vocabulary_query" \
  --epoch "$RUN" --prefix query_000 \
  --calibration "$HOME/e2e_out/calibration.json" \
  --output "$HOME/object_search_out" "$@" || true
echo "=== object_search.json ==="
cat "$HOME/object_search_out/object_search.json" 2>/dev/null || echo "no output"
