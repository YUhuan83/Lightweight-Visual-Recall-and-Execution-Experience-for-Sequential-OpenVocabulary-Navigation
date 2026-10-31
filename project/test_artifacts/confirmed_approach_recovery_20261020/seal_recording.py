"""Package the actual user-requested RViz video after its owned trial closes."""
from pathlib import Path
import hashlib
import json
import tarfile

root = Path(__file__).resolve().parent
trial = root/'full/trial_20261020_225946'
result = json.loads((trial/'result.json').read_text())
assert result['closed'] and result['video'] and result['video_requested']
assert result['video_encoder_exit'] == 0 and result['video_capture']['verified_container']
session = Path(result['session'])
post = json.loads((root/'recorded_poststart.json').read_text())
assert not post['processes'] and not post['display148_lock']

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        while data := f.read(4*1024*1024): h.update(data)
    return h.hexdigest()

files = {name: trial/name for name in ('rviz_live.mp4','result.json','video_probe.json','rviz_preview.png',
    'runtime_identity.json','closed_audit.json','video_encoder.log','console.log')}
files.update({name: session/relative for name,relative in (
    ('scene.json','scene.json'),('stop.json','goal_001/stop.json'),
    ('verified_sources.json','verified_sources.json'),('goal_result.json','goal_001/result.json'),
    ('vision.log','vision.log'),('events.jsonl','goal_001/events.jsonl'))})
files['poststart.json'] = root/'recorded_poststart.json'
files['live_version_check.json'] = trial/'live_version_check.json'
files['frozen_source_manifest.json'] = root/'frozen_source_manifest.json'
files['startup_failure225836.json'] = root/'full/trial_20261020_225836/result.json'
video = files['rviz_live.mp4']; assert sha(video) == result['video_capture']['sha256']
manifest = dict(scope=__doc__, trial=str(trial), files={name: dict(size=p.stat().st_size,sha256=sha(p)) for name,p in files.items()},
    complete_navigation_passed=False,
    source_bag=dict(path=str(session/'goal_001/interfaces/interfaces_0.db3'),
                    sha256=sha(session/'goal_001/interfaces/interfaces_0.db3')))
(root/'recording_delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
archive = root/'recording_delivery_225946.tar'
assert not archive.exists()
with tarfile.open(archive,'w') as t:
    for name,p in files.items(): t.add(p,arcname=name,recursive=False)
    t.add(root/'recording_delivery_manifest.json',arcname='delivery_manifest.json')
report = dict(size=archive.stat().st_size,sha256=sha(archive),files=len(files),
              duration_s=result['video_capture']['duration_s'],video_sha256=sha(video),
              goal_result=result.get('goal_result'),session_errors=result.get('session_result',{}).get('errors'))
(root/'recording_delivery_identity.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
