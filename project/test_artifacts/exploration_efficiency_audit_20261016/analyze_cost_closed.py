from pathlib import Path
import collections,json,math,sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

r=Path('/home/ly/frontier_cost_repair_20261016');field=r/'field';base=Path('/home/ly/wildos-console')
t=sorted(base.glob('field_frontier_cost_search_*'))[-1]
assert (t/'result.json').is_file(),'Driver active; never open an active bag'
trial=json.loads((t/'result.json').read_text());s=Path(trial['session']);g=s/'goal_001'
assert (s/'session_result.json').is_file() and (g/'result.json').is_file(),'Recorders active'
def lines(p):return [json.loads(v) for v in p.read_text().splitlines() if v]
events=lines(g/'events.jsonl');native=lines(g/'native_samples.jsonl')
with sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True) as db:
 def topic(name):
  pair=db.execute('select id,type from topics where name=?',(name,)).fetchone()
  if pair is None:return []
  return [(ts,deserialize_message(v,get_message(pair[1]))) for ts,v in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(pair[0],))]
 decisions=[dict(received_ns=ts,**json.loads(m.data)) for ts,m in topic('/spot1/graphnav_planner/route_status')]
 acks=[dict(received_ns=ts,**json.loads(m.data)) for ts,m in topic('/spot1/graphnav_planner/selection_ack')]
 retired=[dict(received_ns=ts,**json.loads(m.data)) for ts,m in topic('/spot1/graphnav_planner/retired_exploration')]
 choices=[]
 for ts,m in topic('/spot1/graphnav_planner/vehicle_choice'):
  stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  choices.append(dict(received_ns=ts,source_stamp_ns=stamp,identity=m.exploration_id,
   chosen_uuid=m.chosen_uuid,vehicle_cost_m=m.vehicle_cost,
   origin=[m.evaluation_start.position.x,m.evaluation_start.position.y],
   offers=next((d.get('vehicle_options',[]) for d in decisions if d['stamp_ns']==stamp),[])))
unique=list({(v['identity'],v['source_stamp_ns'],v['chosen_uuid']):v for v in choices}.values())
for row in unique:
 row['accepted_acks']=[a for a in acks if a.get('exploration_id')==row['identity'] and a.get('source_stamp_ns')==row['source_stamp_ns']]
 row['selected_decisions']=[d for d in decisions if d.get('vehicle_cost_selected') and d.get('exploration_id','').split(':')[0]==row['chosen_uuid'] and d['stamp_ns']>=row['source_stamp_ns']]
cost_events=[e for e in events if e.get('kind')=='plan_status' and e.get('status') in (
 'exploration_cost_evaluated','exploration_cost_rejected','exploration_cost_unavailable','frontier_selection_waiting','frontier_selection_timeout','recovery_started','recovery_done','recovery_failed','recovery_waiting_for_stop')]
observed=[]
for retirement in retired:
 if retirement.get('reason')!='observation_completed':continue
 ds=[d for d in decisions if d.get('exploration_id')==retirement['exploration_id']]
 if not ds:continue
 begin=ds[0]['received_ns']/1e9;end=retirement['stamp_ns']/1e9
 samples=[v['position'] for v in native if begin<=v['wall_time']<=end and isinstance(v.get('position'),list)]
 observed.append(dict(identity=retirement['exploration_id'],goal=ds[0]['goal'],seconds=end-begin,
  native_distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(samples,samples[1:]))))
stop_review=[]
if (field/'closed_review.json').is_file():
 closed=json.loads((field/'closed_review.json').read_text())
 for stop in closed['stationary_intervals_ge5s']:
  stop_review.append(stop)
report=dict(trial=str(t),session=str(s),goal=json.loads((g/'result.json').read_text()),
 choice_messages=len(choices),unique_choices=unique,selection_ack_count=len(acks),
 selection_ack_reasons=dict(collections.Counter(a.get('reason') for a in acks)),
 selected_ids=sorted({d['exploration_id'] for d in decisions if d.get('vehicle_cost_selected')}),
 observed_transactions=observed,cost_and_recovery_events=cost_events,stationary_windows=stop_review,
 limits='Cost paths themselves were not published as executable full paths; this bag contains cost metrics, typed choices/ACKs, actual route/execution and source map stamps. Do not equate comparison-only paths or cost-selection retirements with observations/arrival.')
(field/'cost_closed_review.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(choice_messages=len(choices),unique_choices=len(unique),selection_ack_reasons=report['selection_ack_reasons'],observed_transactions=observed),indent=2))
