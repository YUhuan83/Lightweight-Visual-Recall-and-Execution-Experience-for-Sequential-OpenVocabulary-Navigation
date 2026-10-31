#!/usr/bin/env bash
set -eo pipefail

D=/home/ly
WS=$D/beamng_ros2_0394_ws
SRC=$WS/src/nebula2-wildos
ARCHIVE=$D/wildos_feature_packages.tar.gz
STAMP=$(date +%Y%m%d-%H%M%S)
BACKUP=$D/wildos_feature_backup_$STAMP

test -f "$ARCHIVE"
mkdir -p "$BACKUP" "$SRC"
for pkg in graphnav_msgs graphnav_mapper graphnav_planner visual_navigation; do
  if test -d "$SRC/$pkg"; then
    cp -a "$SRC/$pkg" "$BACKUP/"
  fi
done

tar -xzf "$ARCHIVE" -C "$SRC"
python3 "$D/prepare_official_wildos.py" \
  --source-root "$SRC" \
  --adapter-root "$WS/src/beamng_wildos_adapter"

bash "$D/wildos_server_stop.sh" >/dev/null 2>&1 || true
pkill -9 -f 'visual_navigation.wildos.nav' 2>/dev/null || true
pkill -9 -f '/install/graphnav_mapper/' 2>/dev/null || true
pkill -9 -f '/install/graphnav_planner/' 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
source "$WS/.venv/bin/activate"
source "$WS/install/setup.bash"
cd "$WS"
colcon build \
  --packages-select graphnav_msgs graphnav_mapper graphnav_planner visual_navigation \
  --symlink-install \
  --event-handlers console_direct+

echo "FEATURE_BUILD_COMPLETE"
echo "BACKUP=$BACKUP"
