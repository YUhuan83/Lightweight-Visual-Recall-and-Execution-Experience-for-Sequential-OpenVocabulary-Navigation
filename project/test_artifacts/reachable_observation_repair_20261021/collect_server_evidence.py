"""Collect compact current-trial evidence; leave the large closed bag on server."""
from pathlib import Path
import json
import tarfile

root=Path('/home/ly/execution_continuity_repair_20261021/reachable_observation_v11')
trial=root/'full/trial_20261021_221131'
session=trial/'sessions/session_20261021_221131'
names=[root/'deployment_receipt.json',root/'installed_identity.json',root/'build.log',
       root/'reachable_ros.log',root/'visible_ros.log',root/'full/frozen_source_manifest.json',
       trial/'result.json',trial/'runtime_identity.json',trial/'prestart.json',trial/'console.log',
       trial/'fluency_audit/closed_trial_summary.json']
for name in ('stack.log','vision.log','triangulation.log','verified_sources.json',
             'scene.json','model_identity.json','session_result.json','sensor_attachment.json'):
    names.append(session/name)
for name in ('events.jsonl','native_samples.jsonl','result.json','stop.json','failure_last_rgb.png'):
    names.append(session/'goal_001'/name)
archive=root/'trial_221131_compact.tar.gz'
with tarfile.open(archive,'w:gz') as bundle:
    for file in names:
        assert file.is_file(),str(file)
        bundle.add(file,arcname=str(file.relative_to(root)))
print(json.dumps(dict(path=str(archive),bytes=archive.stat().st_size,files=len(names))))
