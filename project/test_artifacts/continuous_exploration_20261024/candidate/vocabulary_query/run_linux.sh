#!/usr/bin/env bash
set -euo pipefail
# No installation, download, server access or hardcoded checkpoint drive.
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -lt 5 ]; then
  echo "Usage: bash run_linux.sh SOURCE_ROOT CHECKPOINT_ROOT IMAGE QUERY OUTPUT [extra CLI flags]" >&2
  exit 2
fi
SOURCE_ROOT="$1"; CHECKPOINT_ROOT="$2"; IMAGE="$3"; QUERY="$4"; OUTPUT="$5"
shift 5
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1
exec "${PYTHON:-python3}" -B "$HERE/query.py" --source-root "$SOURCE_ROOT" --checkpoint-root "$CHECKPOINT_ROOT" --image "$IMAGE" --query "$QUERY" --output "$OUTPUT" "$@"
