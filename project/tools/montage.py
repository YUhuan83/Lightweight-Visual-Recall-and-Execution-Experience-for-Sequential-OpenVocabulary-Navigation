#!/usr/bin/env python3
import cv2
import numpy as np

V = "/home/ly/nizhen_runs/nizhen_drive_20260919_163904.mp4"
OUT = "/home/ly/nizhen_frames/montage.png"
cap = cv2.VideoCapture(V)
fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
frames = []
# sample every 2 seconds across the drive (~40s)
step = int(fps * 2)
idx = 0
while True:
    r, f = cap.read()
    if not r:
        break
    if idx % step == 0:
        # crop stamp row + front camera panel (x 0:640)
        crop = f[34:544, 0:640].copy()
        ts = idx / fps
        cv2.putText(crop, "t=%.0fs" % ts, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        frames.append(cv2.resize(crop, (320, 255)))
    idx += 1
cap.release()
cols = 4
rows = (len(frames) + cols - 1) // cols
H = rows * 255
W = cols * 320
canvas = np.zeros((H, W, 3), np.uint8)
for i, im in enumerate(frames):
    r, c = divmod(i, cols)
    canvas[r*255:(r+1)*255, c*320:(c+1)*320] = im
cv2.imwrite(OUT, canvas)
print("frames", len(frames), "->", OUT, canvas.shape)
