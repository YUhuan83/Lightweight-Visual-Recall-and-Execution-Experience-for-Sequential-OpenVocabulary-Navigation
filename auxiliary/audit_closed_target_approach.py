"""Read closed query evidence before attributing approach failures."""
import collections,json,sqlite3,subprocess
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
root=Path('/home/ly/execution_continuity_review_20261022')
check=json.loads(subprocess.run(['python3','/home/ly/planning_phase_physical_20261018/prestart.py'],check=True,capture_output=True,text=True).stdout)
assert not check['processes'] and not check['display148_lock']
records=json.loads((root/'fast_shift_trials_20261024/results.json').read_text())
reports=[]
for record in records:
    trial=Path(record['trial']);assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    out=dict(trial=str(trial),topics={})
    for name in ('/spot1/object_mask','/spot1/object_localization','/spot1/object_confirmation','/spot1/visible_target_approach'):
        tid,typ=topics[name];total=db.execute('select count(*) from messages where topic_id=?',(tid,)).fetchone()[0]
        row=dict(type=typ,count=total)
        if not name.endswith('object_mask'):
            cls=get_message(typ);rows=[]
            for stamp,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
                m=deserialize_message(blob,cls)
                value=json.loads(m.data) if typ=='std_msgs/msg/String' else message_to_ordereddict(m)
                rows.append(dict(recorded_ns=stamp,value=value))
            row['rows']=rows
        out['topics'][name]=row
    db.close()
    events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
    plans={e['stamp_ns']:e for e in events if e.get('kind')=='plan_status'}
    failures={'visible_target_approach','visible_target_local_fallback','visible_target_no_vehicle_path',
        'visible_target_approach_timeout','visible_target_observation_step_done','visible_target_reobserving',
        'visible_target_reobservation_timeout','replanning_after_maneuver','follow_failed','continuous_motion_failed','route_done'}
    out['approach_events']=[e for _,e in sorted(plans.items()) if e.get('status') in failures]
    (trial/'target_approach_audit.json').write_text(json.dumps(out,indent=2))
    reports.append(out)
target=root/'v25_fast_shift/target_approach_audit.json'
target.write_text(json.dumps(reports,indent=2))
print(json.dumps([dict(trial=r['trial'],topics={k:dict(type=v['type'],count=v['count'],examples=[x['value'] for x in v.get('rows',[])[:2]]) for k,v in r['topics'].items()},
    approach_events=[e for e in r['approach_events'] if e['status']!='visible_target_local_fallback']) for r in reports]))
