import cv2
p='test_artifacts/server_validation_20261006/semantic5_retest1/rviz_live.mp4'
c=cv2.VideoCapture(p)
for t in (20,95,195,290,390,425):
 c.set(cv2.CAP_PROP_POS_MSEC,t*1000); ok,f=c.read()
 if ok: cv2.imwrite(f'test_artifacts/server_validation_20261006/semantic5_retest1/rviz{t}.png',f)
