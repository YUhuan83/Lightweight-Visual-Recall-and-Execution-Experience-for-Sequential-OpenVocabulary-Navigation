#!/usr/bin/env bash
set +u
V=/home/ly/nizhen_runs/nizhen_house3_20260919_164834.mp4
mkdir -p /home/ly/nizhen_frames
/home/ly/vision-cu128/bin/python - <<PY
import cv2
V="$V"
cap=cv2.VideoCapture(V)
n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps=cap.get(cv2.CAP_PROP_FPS) or 10.0
print("frames",n,"fps",fps)
for idx,name in [(int(fps*45),"approach"),(int(fps*52),"turn"),(n-1,"final"),(n-15,"final2")]:
    cap.set(cv2.CAP_PROP_POS_FRAMES,max(0,min(idx,n-1)))
    r,f=cap.read()
    if r:
        cv2.imwrite("/home/ly/nizhen_frames/h3_%s.png"%name,f)
        print("wrote",name,f.shape)
cap.release()
PY
ls -la /home/ly/nizhen_frames/h3_*.png
