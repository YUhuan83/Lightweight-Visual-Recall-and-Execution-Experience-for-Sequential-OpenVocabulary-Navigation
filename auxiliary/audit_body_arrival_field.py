"""Inspect only the closed v26 bag; do not infer arrival from a path or mask."""
import collections,hashlib,json,math,sqlite3,subprocess
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.arrival_criteria import vehicle_body_distance
from beamng_wildos_adapter.target_selection import supported_position_clusters
import numpy as np
root=Path('/home/ly/execution_continuity_review_20261022/v26_body_arrival')
trial=root/'full/trial_20261025_145355'
pre=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True).stdout)
assert not pre['processes'] and not pre['display148_lock']
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
counts={n:db.execute('select count(*) from messages where topic_id=?',(topics[n][0],)).fetchone()[0] for n in ('/spot1/object_mask','/spot1/object_localization','/spot1/object_confirmation')}
def load(name):
    tid,typ=topics[name];cls=get_message(typ)
    return [(t,json.loads(deserialize_message(data,cls).data)) for t,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
localizations=load('/spot1/object_localization');confirmations=load('/spot1/object_confirmation')
rows=[];window=[];max_support=0
for received,data in localizations:
    if data.get('localization_valid') is not True:continue
    q=data['observer_orientation'];heading=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    stamp=data['observation_stamp_ns'];window=[v for v in (window+[data])[-10:] if stamp-v['observation_stamp_ns']<=30_000_000_000]
    clusters=supported_position_clusters(np.asarray([v['position'] for v in window]),1.5,1)
    support=max((c[0] for c in clusters),default=0);max_support=max(max_support,support)
    rows.append(dict(recorded_ns=received,stamp_ns=stamp,source=data.get('source'),position=data['position'],observer=data['observer_position'],
        body_distance_at_observation_m=vehicle_body_distance(data['observer_position'],heading,data['position'],'ackermann'),
        source_age_at_record_s=(received-stamp)/1e9,diagnostic_cluster_support=support))
db.close()
preflight=json.loads((goal.parent/'server_preflight.json').read_text())
report=dict(trial=str(trial),counts=counts,valid_localizations=len(rows),max_diagnostic_cluster_support=max_support,
    confirmations=confirmations,localizations=rows,
    caveat='Observation-pose distances and deterministic clustering are diagnostics, not current-pose arrival or semantic ground truth. Original thresholds unchanged.',
    postcheck=pre,preflight=preflight)
encoded=json.dumps(report,sort_keys=True).encode();(root/'arrival_field_audit.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),counts=counts,valid_localizations=len(rows),max_cluster_support=max_support,
    min_body_gap_at_observation_m=min((r['body_distance_at_observation_m'] for r in rows),default=None))))
