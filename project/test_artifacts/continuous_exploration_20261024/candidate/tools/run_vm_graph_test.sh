#!/usr/bin/env bash
set -eo pipefail

PROJECT=/media/sf_wildos_sim
SRC="$PROJECT/wildos_port/repro_check/src/nebula2-wildos"
WS="${WS:-$HOME/wildos_graph_test_ws}"

if [[ ! -d "$SRC/graphnav_mapper" || ! -d "$SRC/graphnav_planner" || ! -d "$SRC/graphnav_msgs" ]]; then
  echo "ERROR: shared WildOS source not found at $SRC" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

PROJECT_REAL="$(realpath -m "$PROJECT")"
SRC_REAL="$(realpath -e "$SRC")"
WS_REAL="$(realpath -m "$WS")"
TARGET="$WS_REAL/src/nebula2-wildos"
if [[ "$WS_REAL" == "/" || "$TARGET" == "$SRC_REAL" \
      || "$SRC_REAL" == "$TARGET/"* || "$TARGET" == "$SRC_REAL/"* ]]; then
  echo "ERROR: refusing unsafe workspace replacement: $TARGET" >&2
  exit 1
fi
if [[ "$WS_REAL" == "$PROJECT_REAL" || "$WS_REAL" == "$PROJECT_REAL/"* \
      || "$PROJECT_REAL" == "$WS_REAL/"* ]]; then
  echo "ERROR: test workspace must be separate from the shared project tree" >&2
  exit 1
fi

LOG_DIR="$WS_REAL/logs"
mkdir -p "$WS_REAL/src" "$LOG_DIR"
STAGED="$WS_REAL/src/.nebula2-wildos.stage.$$"
cp -a "$SRC_REAL" "$STAGED"
if [[ -e "$TARGET" ]]; then
  BACKUP="$WS_REAL/backups/nebula2-wildos.$(date +%Y%m%d%H%M%S).$$"
  mkdir -p "$(dirname "$BACKUP")"
  mv "$TARGET" "$BACKUP"
  echo "PREVIOUS_SOURCE_BACKUP=$BACKUP"
fi
mv "$STAGED" "$TARGET"

echo "SOURCE=$SRC"
echo "WORKSPACE=$WS_REAL"
echo "== source checks =="
grep -R -n -E 'physical_risk|Metadata metadata' \
  "$WS_REAL/src/nebula2-wildos/graphnav_mapper" \
  "$WS_REAL/src/nebula2-wildos/graphnav_planner" | head -n 20 || true

echo "== colcon build =="
cd "$WS_REAL"
colcon build \
  --packages-up-to graphnav_mapper graphnav_planner \
  --event-handlers console_direct+ \
  2>&1 | tee "$LOG_DIR/build.log"

source "$WS_REAL/install/setup.bash"

echo "== colcon test =="
colcon test \
  --packages-up-to graphnav_mapper graphnav_planner \
  --event-handlers console_direct+ \
  2>&1 | tee "$LOG_DIR/test.log"

echo "== test result =="
colcon test-result --verbose | tee "$LOG_DIR/test-result.log"

echo "GRAPH_TEST_COMPLETE"
echo "BUILD_LOG=$LOG_DIR/build.log"
echo "TEST_LOG=$LOG_DIR/test.log"
