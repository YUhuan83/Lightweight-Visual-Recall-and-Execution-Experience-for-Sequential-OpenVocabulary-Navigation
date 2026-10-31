import json
from pathlib import Path
import cv2
r=Path(r'C:\Users\Yu\Desktop\拟真\captures\20261009_B_green_house_station_trial14_obstacle_restore')
e=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\server_validation_20261009\obstacle_restore\server_evidence\trial14')
rows=[json.loads(s) for s in (e/'front_camera_frames.jsonl').read_text().splitlines()]
target=1791513863084888339
selected=sorted(rows,key=lambda x:abs(x['rgb_stamp_ns']-target))[:3]
c=cv2.VideoCapture(str(r/'front_camera.mp4'))
for i,row in enumerate(selected):
 c.set(cv2.CAP_PROP_POS_FRAMES,row['frame_index']);ok,f=c.read();assert ok
 _,b=cv2.imencode('.jpg',f);(r/'review'/('arrival_measurement_'+str(i)+'.jpg')).write_bytes(b.tobytes())
 print(i,{k:row.get(k) for k in ('elapsed','frame_index','rgb_stamp_ns','odom_yaw','native_direction','overlay_applied')},'skew',(row['rgb_stamp_ns']-target)/1e9)
c.release()
(r/'review'/'arrival_measurement_frame_index.json').write_text(json.dumps(selected,indent=2))
