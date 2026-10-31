#!/usr/bin/env bash
set +u
export DISPLAY=:3
export XAUTHORITY=/run/user/1011/gdm/Xauthority
echo "== xrandr =="
timeout 10 xrandr 2>&1 | head -n 5
echo "== window tools =="
which wmctrl xdotool xwininfo xprop 2>/dev/null
echo "== root children =="
timeout 10 xwininfo -root -children 2>&1 | head -n 40
echo "== rviz proc =="
pgrep -af rviz2 | grep -v pgrep
echo "== xlsclients =="
timeout 10 xlsclients 2>&1 | head
