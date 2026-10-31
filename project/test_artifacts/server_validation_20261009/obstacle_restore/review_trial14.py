"""Read stopped trial evidence and extract frames using its real time index."""
import hashlib
import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
EVIDENCE = ROOT / 'server_evidence' / 'trial14'
CAPTURE = ROOT.parents[2] / 'captures' / '20261009_B_green_house_station_trial14_obstacle_restore'
REVIEW = CAPTURE / 'review'
REVIEW.mkdir(exist_ok=True)
result = json.loads((EVIDENCE / 'full_test.json').read_text(encoding='utf8'))
manifest = json.loads((EVIDENCE / 'video_manifest.json').read_text(encoding='utf8'))
frames = [json.loads(s) for s in (EVIDENCE / 'front_camera_frames.jsonl').read_text(encoding='utf8').splitlines()]
events = [json.loads(s) for s in (EVIDENCE / 'events.jsonl').read_text(encoding='utf8').splitlines()]
report = {'scope': 'Video integrity and indexed visual review; original automatic result is retained.', 'automatic_passed': result['passed'], 'videos': {}, 'frames': []}
for name in ('front_camera.mp4', 'rviz_live.mp4'):
    path = CAPTURE / name
    if not path.exists():
        continue
    assert path.stat().st_size == manifest[name]['bytes'], name + ' incomplete transfer'
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest == manifest[name]['sha256'], name + ' hash mismatch'
    cap = cv2.VideoCapture(str(path))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
    ok, _ = cap.read()
    assert ok, name + ' last frame decode failed'
    report['videos'][name] = {'bytes': path.stat().st_size, 'sha256': digest, 'frames': count, 'fps': fps, 'duration_s': count / fps, 'last_frame_decoded': ok}
    cap.release()

house_elapsed = next(x['elapsed'] for x in events if x['kind'] == 'green_house_search_finished')
station_elapsed = max(x['elapsed'] for x in events if x['phase'] == 'station_return')
cap = cv2.VideoCapture(str(CAPTURE / 'front_camera.mp4'))
for label, elapsed in [('house_approach', house_elapsed - 15), ('house_arrival', house_elapsed), ('station_approach', station_elapsed - 15), ('station_arrival', station_elapsed)]:
    row = min(frames, key=lambda x: abs(x['elapsed'] - elapsed))
    cap.set(cv2.CAP_PROP_POS_FRAMES, row['frame_index'])
    ok, frame = cap.read()
    assert ok, label
    path = REVIEW / (label + '.jpg')
    encoded_ok, encoded = cv2.imencode('.jpg', frame)
    assert encoded_ok, label + ' JPEG encoding failed'
    path.write_bytes(encoded.tobytes())
    report['frames'].append({'label': label, 'requested_elapsed_s': elapsed, 'index': row['frame_index'], 'video_pts_sec': row['video_pts_sec'], 'actual_elapsed_s': row['elapsed'], 'rgb_stamp_ns': row['rgb_stamp_ns'], 'query': row.get('query'), 'image': str(path)})
cap.release()
(REVIEW / 'video_review.json').write_text(json.dumps(report, indent=2), encoding='utf8')
print(json.dumps(report, indent=2))
