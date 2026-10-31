#!/usr/bin/env bash
set +u
export DISPLAY=:3
export XAUTHORITY=/run/user/1011/gdm/Xauthority
echo "== disable screensaver =="
pkill -9 -f xfce4-screensaver 2>/dev/null || true
timeout 15 xset s off 2>&1 || true
timeout 15 xset -dpms 2>&1 || true
sleep 2
echo "== all windows mentioning rviz or sized 1920 =="
timeout 10 xwininfo -root -children 2>&1 | grep -iE 'rviz|1920x|1280x|1600x' | head -n 40
echo "== rviz process threads =="
pgrep -af rviz2 | grep -v pgrep
echo "== recapture =="
timeout 20 scrot -o /tmp/rviz_screen2.png 2>&1 && echo SCROT_OK || echo SCROT_FAIL
ls -la /tmp/rviz_screen2.png 2>/dev/null
