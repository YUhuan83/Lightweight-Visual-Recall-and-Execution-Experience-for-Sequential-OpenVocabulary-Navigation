"""Verify and deliver the actual 221250 partial RViz recording without relabeling success."""
from pathlib import Path
import hashlib
import json
import tarfile
import cv2
import numpy as np

root=Path(__file__).resolve().parent;repo=root.parents[1]
archive=root/'recording_delivery_221250.tar'
assert archive.stat().st_size==31477760
assert hashlib.sha256(archive.read_bytes()).hexdigest()==('93be5d56f00921c2613c56299eb10d31'+'37fc023816b01750f984759a46d37230')
out=repo/'captures/20261020_green_house_rviz_trial221250';out.mkdir(exist_ok=True)
allowed={'rviz_live.mp4','rviz_preview.png','result.json','video_probe.json','video_encoder.log','rviz_window.txt'}
with tarfile.open(archive) as tar:
    members=tar.getmembers();assert len(members)==len(allowed)
    for item in members:
        assert item.isfile() and item.name in allowed
        data=tar.extractfile(item).read();target=out/item.name
        if target.exists():assert target.read_bytes()==data
        else:target.write_bytes(data)
result=json.loads((out/'result.json').read_text());video=out/'rviz_live.mp4'
assert hashlib.sha256(video.read_bytes()).hexdigest()==result['video_capture']['sha256']
assert result['closed'] and result['goal_result']['state']=='session_exit' and not result['harness_arrival']
assert 'control lease expired or closing' in result['goal_result']['failure_reason']
cap=cv2.VideoCapture(str(video));assert cap.isOpened()
fps=cap.get(cv2.CAP_PROP_FPS);count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
assert fps==10 and count==476
frames=[]
for seconds in (5,20,40):
    cap.set(cv2.CAP_PROP_POS_MSEC,seconds*1000);ok,frame=cap.read();assert ok
    assert frame.shape[:2]==(1080,1920)
    encoded_ok,encoded=cv2.imencode('.png',frame);assert encoded_ok
    (out/('frame_%02ds.png'%seconds)).write_bytes(encoded.tobytes())
    frames.append(frame)
cap.release()
# Visible content change is checked separately from claimed container FPS.
camera_differences=[float(np.mean(np.abs(a[100:1030,990:1900].astype(float)-b[100:1030,990:1900].astype(float)))) for a,b in zip(frames,frames[1:])]
assert all(d>1. for d in camera_differences),camera_differences
report=dict(verified=True,video=str(video),sha256=result['video_capture']['sha256'],duration_s=count/fps,
            width=1920,height=1080,fps=fps,frames=count,camera_pixel_mean_differences=camera_differences,
            search_elapsed_s=result['goal_result']['elapsed_s'],state='failed_control_lease',
            complete_navigation_passed=False,scope='Actual partial RViz window recording; not the preceding 420s trial.')
(out/'delivery_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
