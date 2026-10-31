#!/usr/bin/env bash
set +u
V=/home/ly/nizhen_runs/nizhen_drive_20260919_163557.mp4
echo "== recorder log =="
grep -iE 'error|fail|opened|recorded' /tmp/video_recorder.log 2>/dev/null | tail -n 8
echo "== ffprobe =="
which ffprobe ffmpeg 2>/dev/null
ffprobe -v error -select_streams v:0 -show_entries stream=width,height,nb_frames,avg_frame_rate,codec_name -of default=nk=1:nw=1 "$V" 2>&1 | head -n 10
echo "== cv2 probe =="
/home/ly/vision-cu128/bin/python - <<PY
import cv2
cap=cv2.VideoCapture("$V")
print("opened", cap.isOpened())
n=0
import os
ok=True
while True:
    r,f=cap.read()
    if not r: break
    n+=1
    if n==1: print("first frame shape", None if f is None else f.shape)
print("frames", n)
cap.release()
PY
