#!/usr/bin/env bash
# Recorded full-trial launcher (v2).
# Changes vs v1:
#  - Recording starts only after the RViz window is mapped AND the display
#    is non-black (YAVG > REC_MIN_YAVG). v1 recorded the empty Xvfb screen for
#    the first ~30 s of every video.
#  - Recording stops when the goal result is written (before RViz shutdown),
#    and ffmpeg is stopped with SIGTERM. v1 stopped it up to 20 s after RViz
#    exited, producing a black tail.
#  - Capture rate is configurable (REC_FPS, default 20) instead of 10.
set -u
BASE=/home/ly/execution_continuity_repair_20261021/v10
START_EPOCH=$(date +%s)
LOG="$BASE/full/recorded_driver_launcher.log"
REC_FPS="${REC_FPS:-20}"
REC_MIN_YAVG="${REC_MIN_YAVG:-24}"
DISPLAY_NUM=":148"
bash "$BASE/run_full.sh" >"$LOG" 2>&1 &
runner=$!
recorder=""
trial=""

display_yavg() {
  # One frame from the field display; prints lavfi.signalstats.YAVG.
  ffmpeg -nostdin -hide_banner -loglevel quiet -f x11grab -framerate 5 \
    -video_size 1920x1080 -i "${DISPLAY_NUM}.0" -frames:v 1 \
    -vf signalstats,metadata=print:file=- -f null - 2>/dev/null \
    | sed -n 's/.*lavfi\.signalstats\.YAVG=//p' | tail -1
}

rviz_ready() {
  # The blank Xvfb root measures YAVG=16 (limited range); any rendered RViz
  # scene is well above the threshold.  The window name is not relied on.
  local y
  y=$(display_yavg)
  [ -n "$y" ] || return 1
  awk -v y="$y" -v t="$REC_MIN_YAVG" 'BEGIN{exit !(y>t)}'
}

stop_recorder() {
  [ -n "$recorder" ] || return 0
  kill -TERM "$recorder" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$recorder" 2>/dev/null || break
    sleep 0.5
  done
  kill -KILL "$recorder" 2>/dev/null || true
  wait "$recorder" 2>/dev/null || true
  recorder=""
}

deadline=$((START_EPOCH+720))
while [ "$(date +%s)" -lt "$deadline" ]; do
  if [ -z "$trial" ]; then
    trial=$(find "$BASE/full" -maxdepth 1 -type d -name 'trial_*' -newermt "@$START_EPOCH" -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)
  fi
  if [ -n "$trial" ] && [ -S /tmp/.X11-unix/X148 ] && [ -z "$recorder" ] && rviz_ready; then
    ffmpeg -nostdin -hide_banner -loglevel warning -f x11grab -framerate "$REC_FPS" \
      -video_size 1920x1080 -i "${DISPLAY_NUM}.0" -an -c:v libx264 -preset ultrafast \
      -crf 25 -pix_fmt yuv420p -threads 2 -movflags +faststart \
      "$trial/rviz_live.mp4" >"$trial/ffmpeg_record.log" 2>&1 &
    recorder=$!
    echo "RECORDING trial=$trial pid=$recorder fps=$REC_FPS started=$(date -u +%FT%TZ)"
  fi
  # The goal result is written before RViz shuts down; stop capture there
  # instead of after the display has already gone black.
  if [ -n "$trial" ] && [ -n "$recorder" ] && \
     compgen -G "$trial/sessions/session_*/goal_*/result.json" >/dev/null; then
    sleep 0.5
    stop_recorder
    echo "RECORDING_STOPPED trial=$trial at=$(date -u +%FT%TZ)"
  fi
  if ! kill -0 "$runner" 2>/dev/null; then break; fi
  sleep 1
done
if kill -0 "$runner" 2>/dev/null; then
  echo "DRIVER_TIMEOUT" >&2
  kill -TERM "$runner" 2>/dev/null || true
fi
wait "$runner"; status=$?
stop_recorder
# If the goal result never appeared, the recording still stops here.
printf 'DRIVER_STATUS=%s\nTRIAL=%s\n' "$status" "$trial"
[ -n "$trial" ] && { cat "$trial/result.json" 2>/dev/null || true; ls -lh "$trial/rviz_live.mp4" 2>/dev/null || true; }
exit "$status"
