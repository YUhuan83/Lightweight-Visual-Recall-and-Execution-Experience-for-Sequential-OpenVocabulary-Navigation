"""Export bounded synchronized evidence from the CLOSED 090814 bags only."""
from pathlib import Path
import hashlib,json,sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from wildos_nav2_adapter.pointcloud import decode_xyz
ROOT=Path('/home/ly/controller_source_field_20261018')
OUT=Path('/home/ly/execution_wait_audit_20261018');OUT.mkdir(exist_ok=True)
session=ROOT/'field_results/session_20261018_090814';goal=session/'goal_001'
assert (session/'session_result.json').is_file() and (goal/'result.json').is_file()
assert (ROOT/'field_yaw_left90_search_20261018_090810/result.json').is_file()
manifest=json.loads((ROOT/'manifest.json').read_text())
for name,sha in manifest['members'].items():
    assert hashlib.sha256((ROOT/'candidate'/name).read_bytes()).hexdigest()==sha,name
bags={label:sqlite3.connect(next(folder.glob('*.db3')).as_uri()+'?mode=ro',uri=True)
      for label,folder in (('interfaces',goal/'interfaces'),('sensors',session/'sensor_interfaces'))}
topics={label:{name:(i,kind) for i,name,kind in c.execute('select id,name,type from topics')}
        for label,c in bags.items()}
def before(label,topic,ns):
    c=bags[label];pair=topics[label].get(topic)
    if not pair:return None
    row=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(pair[0],ns)).fetchone()
    return (row[0],deserialize_message(row[1],get_message(pair[1]))) if row else None
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines() if x]
stalls=[e for e in events if e.get('reason')=='execution_stalled']
report=dict(candidate=manifest['sha256'],source_members_verified=len(manifest['members']),
    scope='Public maps, source/filtered clouds and TF from closed bags; not the private costmap at controller computation',samples=[])
clouds={'markings':'/wildos/points','clearing':'/wildos/clearing_points','adapted':'/spot1/ouster/front/points_filtered'}
for index,event in enumerate(stalls):
    for offset in (-7.,-5.,-1.,0.):
        target_ns=event['stamp_ns']+round(offset*1e9)
        label=f'case{index}_{int(-offset)}s_before'
        arrays={};sample=dict(label=label,event=event,target_ns=target_ns,clouds={},maps={})
        odom=before('interfaces','/spot1/odom',target_ns)
        sample['odom']=dict(received_ns=odom[0],message=message_to_ordereddict(odom[1]))
        for key,topic in clouds.items():
            found=before('sensors',topic,target_ns)
            if not found:continue
            ts,msg=found;points=decode_xyz(msg)
            arrays[key]=points
            sample['clouds'][key]=dict(topic=topic,received_ns=ts,
                stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                frame=msg.header.frame_id,count=len(points))
        for key,topic in (('local','/local_costmap/costmap'),('global','/global_costmap/costmap')):
            found=before('interfaces',topic,target_ns)
            if not found:continue
            ts,msg=found
            arrays[key]=np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
            sample['maps'][key]=dict(received_ns=ts,header=message_to_ordereddict(msg.header),
                info=message_to_ordereddict(msg.info))
        tfrows=[]
        for topic in ('/tf','/tf_static'):
            pair=topics['sensors'][topic]
            rows=bags['sensors'].execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 160',(pair[0],target_ns))
            seen=set()
            for ts,data in rows:
                msg=deserialize_message(data,get_message(pair[1]))
                for transform in msg.transforms:
                    child=transform.child_frame_id
                    if child in seen:continue
                    seen.add(child);tfrows.append(dict(received_ns=ts,message=message_to_ordereddict(transform)))
        sample['transforms']=tfrows
        np.savez_compressed(OUT/(label+'.npz'),**arrays)
        report['samples'].append(sample)
for c in bags.values():c.close()
(OUT/'map_evidence.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(samples=len(report['samples']),files=[p.name for p in OUT.glob('*.npz')],candidate=manifest['sha256']),indent=2))
