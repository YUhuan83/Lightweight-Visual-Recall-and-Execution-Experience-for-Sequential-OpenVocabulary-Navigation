#!/usr/bin/env bash
set +u
V=/home/ly/nizhen_runs/nizhen_drive_20260919_163904.mp4
mkdir -p /home/ly/nizhen_frames
/home/ly/vision-cu128/bin/python - <<PY
import cv2
V="$V"
cap=cv2.VideoCapture(V)
fps=cap.get(cv2.CAP_PROP_FPS) or 10.0
n=cap.get(cv2.CAP_PROP_FRAME_COUNT)
print("fps",fps,"frames",n)
want={0:"t00", int(fps*12):"t12", int(fps*25):"t25", int(fps*38):"t38", int(n)-1:"tend"}
i=0
while True:
    r,f=cap.read()
    if not r: break
    if i in want:
        p="/home/ly/nizhen_frames/%s.png"%want[i]
        cv2.imwrite(p,f); print("wrote",p)
    i+=1
cap.release()
PY
ls -la /home/ly/nizhen_frames/
