"""Read the sealed bag, preserving typed interface counterexamples."""
import collections,json,math,sqlite3,statistics
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate/green_house_trial1')
events=[json.loads(l) for l in (ROOT/'events.jsonl').read_text().splitlines()]
db=next((ROOT/'interfaces').glob('*.db3'))
connection=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
assert connection.execute('PRAGMA integrity_check').fetchone()[0]=='ok'
topics={name:(tid,get_message(typ)) for tid,name,typ in connection.execute('SELECT id,name,type FROM topics')}
def stamp(header):return header.stamp.sec*10**9+header.stamp.nanosec
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def path(msg):return {'stamp_ns':stamp(msg.header),'frame_id':msg.header.frame_id,
    'points':[[p.pose.position.x,p.pose.position.y] for p in msg.poses],
    'headings':[yaw(p.pose.orientation) for p in msg.poses]}
def messages(topic,lo=None,hi=None):
    if topic not in topics:return
    tid,kind=topics[topic];query='SELECT timestamp,data FROM messages WHERE topic_id=?';args=[tid]
    if lo is not None:query+=' AND timestamp>=?';args.append(lo)
    if hi is not None:query+=' AND timestamp<=?';args.append(hi)
    for received,blob in connection.execute(query+' ORDER BY timestamp',args):
        yield received,deserialize_message(blob,kind)
report={'scope':'Full failed single-target typed bag audit; no physical replay',
    'localization':[],'visible_approaches':[],'masks':[],'geometry_rejections':[]}
for name,key in (('/spot1/object_localization','localization'),('/spot1/visible_target_approach','visible_approaches')):
    for received,msg in messages(name):report[key].append({'received_ns':received,'payload':json.loads(msg.data)})
for received,msg in messages('/spot1/object_mask'):
    report['masks'].append({'received_ns':received,'stamp_ns':stamp(msg.header),'query':msg.query,
        'odom_stamp_ns':stamp(msg.odom.header),'frame_id':msg.header.frame_id,
        'mask_dimensions':[{ 'label':d.label,'size':d.size,'stride':d.stride} for d in msg.object_mask.layout.dim],
        'nonzero_pixels':sum(v!=0 for v in msg.object_mask.data),
        'camera_info':[{'stamp_ns':stamp(c.header),'width':c.width,'height':c.height,'k':list(c.k)} for c in msg.cam_infos],
        'camera_transforms':[{'stamp_ns':stamp(t.header),'parent':t.header.frame_id,'child':t.child_frame_id} for t in msg.cam_transforms.transforms]})
ages=[];latest_odom=None;fresh_external_stale=0
selected=[topics[t][0] for t in ('/spot1/odom','/received_global_plan')]
for received,tid,blob in connection.execute('SELECT timestamp,topic_id,data FROM messages WHERE topic_id IN (?,?) ORDER BY timestamp',selected):
    topic=next(t for t in ('/spot1/odom','/received_global_plan') if topics[t][0]==tid)
    msg=deserialize_message(blob,topics[topic][1])
    if topic=='/spot1/odom':latest_odom=stamp(msg.header)
    else:
        age=(received-stamp(msg.header))/1e9;ages.append(age)
        if age>.5 and latest_odom and (received-latest_odom)/1e9<.3:fresh_external_stale+=1
report['controller_pose_age']={'count':len(ages),'max':max(ages),'p95':sorted(ages)[int(.95*(len(ages)-1))],
    'median':statistics.median(ages),'stale_with_fresh_external_odom':fresh_external_stale}
for event in events:
    if not (event.get('reason')=='excess_curvature' or event.get('detail')=='excess_curvature'):continue
    at=event['stamp_ns']
    original=[{'received_ns':r,**path(m)} for r,m in messages('/plan',at-60*10**9,at)]
    sections=[{'received_ns':r,**path(m)} for r,m in messages('/wildos/planned_path',at-60*10**9,at)]
    report['geometry_rejections'].append({'event':event,'original_plans':original,'published_sections':sections})
    if len(report['geometry_rejections'])>=3:break
(ROOT/'typed_interface_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'controller_pose_age':report['controller_pose_age'],
    'localization':report['localization'],'mask_count':len(report['masks']),
    'visible_approach_count':len(report['visible_approaches']),
    'geometry_rejection_count':len(report['geometry_rejections'])},indent=2))
