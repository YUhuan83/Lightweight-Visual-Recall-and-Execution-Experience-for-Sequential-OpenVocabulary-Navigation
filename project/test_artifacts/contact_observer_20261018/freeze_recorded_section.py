"""Freeze the historical short section and measured start state; no commands."""
from pathlib import Path
import hashlib
import json
import math

root=Path(__file__).resolve().parents[2]
session=root/'test_artifacts/smac_full_field_20261018/closed_evidence/session'
snapshot=root/'test_artifacts/system_connectivity_audit_20261018/stall_connectivity_164222/section_dispatched.json'
path=json.loads(snapshot.read_text())['path']
stamp=path['header']['stamp']['sec']*10**9+path['header']['stamp']['nanosec']
end=path['poses'][-1]['pose']['position']
statuses={};native=[]
for line in (session/'execution_observer/messages.jsonl').open(encoding='utf8'):
    row=json.loads(line)
    if row['topic']=='/wildos/plan_status':
        value=json.loads(row['message']['data'])
        if (value['status']=='planned' and abs(value['stamp_ns']-stamp)<100_000_000
                and math.dist(value['goal'],[end['x'],end['y']])<1e-5):
            statuses[value['stamp_ns']]=value
    elif row['topic']=='/wildos/native_control_status':
        value=json.loads(row['message']['data'])
        if abs(value['wall_time']-stamp/1e9)<.15:native.append(value)
assert len(statuses)==1 and native
start=min(native,key=lambda n:abs(n['wall_time']-stamp/1e9))
fixture=dict(scope=__doc__,path=path,original_planned_status=next(iter(statuses.values())),
    nearby_measured_native_state=start,
    path_snapshot_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    historical_database_sha256='dac5d8cfeef67a2a71b72edb324b4f5f80f994749d386a01b5a7a74f33c4ad45',
    frozen_candidate='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b',
    spawn_pose_reconstruction_validated=False, moving_replay_run=False,
    limits='Frozen evidence only. Fresh spawn lacks historical deformation and prior gear/velocity state; any later replay is a controlled component, not exact full-query replay or visual-arrival acceptance.')
out=Path(__file__).parent/'recorded_section_fixture.json'
out.write_text(json.dumps(fixture,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(path_poses=len(path['poses']),goal_checker=fixture['original_planned_status']['goal_checker_id'],
                     actual_start=fixture['original_planned_status']['actual_start_pose'],
                     native_speed=start['speed'],moving_replay_run=False),indent=2))
