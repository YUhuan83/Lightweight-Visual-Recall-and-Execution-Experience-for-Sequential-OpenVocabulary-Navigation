"""Closed-recording producer replay; actual maps/poses retained, no vehicle simulation."""
import collections,hashlib,json,sqlite3,sys,time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.goal_standoff import GoalStandoff
base=Path('/home/ly/execution_continuity_review_20261022');runtime=base/sys.argv[1];evidence=base/'v47_depth_geometry'
r=json.loads((evidence/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
goal=next(Path(r['trial']).glob('sessions/*/goal_001'));db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
callbacks={'/spot1/odom':'on_odom','/spot1/local_map':'on_occupancy','/global_costmap/costmap':'on_vehicle_grid','/spot1/object_mask':'on_visible_target','/spot1/object_localization':'on_localization'}
ids=[i for i,(n,_) in topics.items() if n in callbacks]
rclpy.init();n=GoalStandoff();now=0;n.get_clock=lambda:S(now=lambda:S(nanoseconds=now,to_msg=lambda:Time(sec=now//10**9,nanosec=now%10**9)))
n.get_logger=Mock();n.active_query='green residential house';n.query_started_ns=0;n.confirmation_window_sec=60.;n.observation_minimum_turning_radius=12.
n.confirmation_pub=Mock();n.publisher=Mock();sent=[];context={};n.visible_approach_pub=Mock()
def publish(m):
 d=json.loads(m.data);sent.append(dict(received_ns=now,trigger=context.get('topic'),**d))
n.visible_approach_pub.publish.side_effect=publish
samples=[];last_tick=0
try:
 for topic_id,t,blob in db.execute('select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in ids)+') order by timestamp,id',ids):
  now=t;name,cls=topics[topic_id];m=deserialize_message(blob,cls);context['topic']=name
  getattr(n,callbacks[name])(m)
  if name=='/spot1/object_localization':
   d=json.loads(m.data);samples.append(dict(stamp_ns=d['observation_stamp_ns'],range_valid=d.get('localization_valid'),visible_stamp=n.visible_target['stamp'] if n.visible_target else None,visible_count=n.visible_target_count,latest=n.observations[-1]['observation_stamp_ns'] if n.observations else None))
  if t-last_tick>=200_000_000:
   context['topic']='timer';n.on_timer();last_tick=t
 source=Path(sys.modules[GoalStandoff.__module__].__file__)
 out=dict(runtime=runtime.name,producer_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),scope='Recorded delivery order and original maps/poses, actual producer callbacks; timer sampled at messages, no counterfactual vehicle/actions/failures simulated.',range_measurements=samples,counts=dict(collections.Counter(d['evidence'] for d in sent)),confirmations=n.confirmation_pub.publish.call_count,proposals=sent)
 (runtime/'range_candidate_replay.json').write_text(json.dumps(out,indent=2,sort_keys=True));print(json.dumps({k:v for k,v in out.items() if k not in ('proposals','range_measurements')},indent=2))
finally:n.destroy_node();rclpy.shutdown();db.close()
