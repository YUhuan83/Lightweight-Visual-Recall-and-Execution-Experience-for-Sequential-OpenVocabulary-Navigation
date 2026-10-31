import cv2,numpy as np
p='test_artifacts/server_validation_20261006/semantic5_retest1/front_camera.mp4'
c=cv2.VideoCapture(p); out=[]; i=0
while True:
 ok,f=c.read()
 if not ok: break
 b,g,r=cv2.split(f)
 m=(g>95)&(g.astype(np.int16)>r.astype(np.int16)*1.25)&(g.astype(np.int16)>b.astype(np.int16)*1.25)
 count=int(m.sum())
 if count>200: out.append((round(i/5,1),count))
 i+=1
print('frames',i,'overlay_frames',len(out),'times_counts',out[:40])
