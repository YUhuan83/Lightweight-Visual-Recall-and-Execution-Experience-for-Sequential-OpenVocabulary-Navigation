"""Verify copied video hashes, container metadata and changing camera samples."""
from pathlib import Path
import hashlib
import json
import tarfile
import cv2
import numpy as np

root = Path(__file__).resolve().parent; repo = root.parents[1]
identity = json.loads((root/'recording_delivery_identity.json').read_text())
archive = root/'recording_delivery_225946.tar'
assert archive.stat().st_size == identity['size']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == identity['sha256']
out = repo/'captures/20261020_confirmed_recovery_trial225946'
out.mkdir(exist_ok=False)
with tarfile.open(archive) as t:
    manifest = json.load(t.extractfile('delivery_manifest.json'))
    assert len(t.getmembers()) == len(manifest['files'])+1
    for name, item in manifest['files'].items():
        assert Path(name).name == name and name not in ('.','..')
        data = t.extractfile(name).read()
        assert len(data) == item['size'] and hashlib.sha256(data).hexdigest() == item['sha256'], name
        (out/name).write_bytes(data)
    (out/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
video = out/'rviz_live.mp4'
cap = cv2.VideoCapture(str(video)); assert cap.isOpened()
fps = cap.get(cv2.CAP_PROP_FPS); frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); duration = frames/fps
assert fps == 10 and frames > 100
probe = json.loads((out/'video_probe.json').read_text()); stream = next(s for s in probe['streams'] if s['codec_type']=='video')
assert frames == int(stream['nb_frames']) and abs(duration-float(probe['format']['duration'])) < .2
samples = []; pictures = []
for sec in sorted(set([5., min(60.,duration/3), min(120.,duration*2/3), max(1.,duration-5.)])):
    cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000); ok,frame = cap.read(); assert ok
    assert frame.shape[:2] == (1080,1920)
    samples.append(dict(second=sec, mean=float(frame.mean())))
    pictures.append(frame[250:880,975:1900].astype(np.int16))
    encoded, data = cv2.imencode('.png',frame); assert encoded
    (out/f'frame_{sec:.1f}s.png').write_bytes(data.tobytes())
cap.release()
changes = [float(np.abs(b-a).mean()) for a,b in zip(pictures,pictures[1:])]
assert max(changes) > 1., 'Sampled camera region unchanged; inspect actual motion and image updates'
result = json.loads((out/'result.json').read_text()); assert result['closed'] and result['video']
assert not json.loads((out/'poststart.json').read_text())['processes']
report = dict(verified=True,video=str(video),video_sha256=manifest['files']['rviz_live.mp4']['sha256'],
              duration_s=duration,fps=fps,frames=frames,samples=samples,camera_sample_differences=changes,
              complete_navigation_passed=False,goal_result=result.get('goal_result'))
(out/'delivery_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
