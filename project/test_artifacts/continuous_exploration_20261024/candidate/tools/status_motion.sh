#!/usr/bin/env bash
set +u
echo "=== motion log tail ==="
tail -n 30 /tmp/motion_run.log 2>/dev/null || echo no_log
echo "=== procs ==="
for p in 'BeamNG.tech.x64 -nosteam' 'visual_navigation.wildos.nav' 'video_recorder.py' 'cmd_vel_to_control' 'path_planner_bridge' 'run_motion_video.sh'; do
  if pgrep -f "$p" >/dev/null 2>&1; then echo "ALIVE  $p"; else echo "DEAD   $p"; fi
done
echo "=== video recorder log ==="
tail -n 8 /tmp/video_recorder.log 2>/dev/null || echo no_rec_log
echo "=== videos ==="
ls -la /home/ly/nizhen_runs/ 2>/dev/null || echo no_dir
