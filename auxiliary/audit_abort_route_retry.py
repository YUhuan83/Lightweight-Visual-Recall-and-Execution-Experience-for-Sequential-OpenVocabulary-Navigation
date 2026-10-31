import json,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import distance_to_route
from wildos_nav2_adapter.graph_path_bridge import xy
root=Path('/home/ly/execution_continuity_review_20261022')/(sys.argv[1] if len(sys.argv)>1 else 'v31_confirmed_progress')
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def rows(name):
    idx,cls=topics[name]
    return [deserialize_message(b,cls) for b, in db.execute('select data from messages where topic_id=? order by timestamp',(idx,))]
plans=[json.loads(m.data) for m in rows('/wildos/plan_status')]
paths={m.header.stamp.sec*10**9+m.header.stamp.nanosec:[xy(p.pose) for p in m.poses] for m in rows('/spot1/graphnav_planner/path')}
report=[]
for index,failed in enumerate(plans):
    if failed.get('status')!='follow_failed':continue
    resumed=next((p for p in plans[index+1:] if p.get('status')=='following'),None)
    if resumed is None:continue
    old=paths.get(failed['replay_inputs']['route_stamp_ns']);new=paths.get(resumed['replay_inputs']['route_stamp_ns'])
    deviation=max((distance_to_route(p,old) for p in new),default=0.) if old and new else None
    delta=(resumed['stamp_ns']-failed['stamp_ns'])*1e-9
    report.append(dict(failed_ns=failed['stamp_ns'],resumed_ns=resumed['stamp_ns'],gap_sec=delta,
        retry_after_sec=failed.get('retry_after_sec'),same_owner=failed.get('exploration_id')==resumed.get('exploration_id'),
        route_max_deviation_m=deviation,new_route_can_earn_early_retry=deviation>3. if deviation is not None else None,
        failed_route_stamp=failed['replay_inputs']['route_stamp_ns'],resumed_route_stamp=resumed['replay_inputs']['route_stamp_ns']))
db.close();(root/'abort_retry_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
