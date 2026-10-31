"""Closed-bag production-method replay; no DDS publishers or vehicle process."""
import collections, hashlib, json, sqlite3, types
from pathlib import Path
import numpy as np
import yaml
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from visual_navigation.explorfm_triangulation.obj_mask_triangulation import ObjectMaskTriangulator as Node
from visual_navigation.explorfm_triangulation import localization_geometry as geometry

ROOT=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
OUT=Path('/home/ly/execution_continuity_review_20261022/target_contract_20261025')
OUT.mkdir(exist_ok=True)
cfg=yaml.safe_load((ROOT/'candidate/official_adapter/config/beamng_triangulation.yaml').read_text())
def ns(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def clean(v):
    if isinstance(v,np.ndarray): return v.tolist()
    if isinstance(v,np.generic): return v.item()
    if isinstance(v,dict): return {k:clean(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)): return [clean(x) for x in v]
    return v

results=[]
for trial in sorted((ROOT/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    def messages(name):
        tid,typ=topics[name];cls=get_message(typ)
        for bt,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
            yield bt,deserialize_message(data,cls)
    paired={d['mask_stamp_ns']:d for _,m in messages('/spot1/measurement_timing') for d in [json.loads(m.data)] if d.get('sensor')=='mask_lidar_sync' and d.get('status')=='paired'}
    wanted={d['lidar_stamp_ns'] for d in paired.values()}
    clouds={ns(m):m for _,m in messages('/spot1/ouster/front/points_filtered') if ns(m) in wanted}
    masks=list(messages('/spot1/object_mask'))
    actual={d['observation_stamp_ns']:d for _,m in messages('/spot1/object_localization') for d in [json.loads(m.data)]}
    dynamics=[(bt,tf) for bt,m in messages('/tf') for tf in m.transforms]
    statics=[(bt,tf) for bt,m in messages('/tf_static') for tf in m.transforms]
    static_groups=collections.defaultdict(list)
    for bt,tf in statics: static_groups[tf.child_frame_id].append((bt,str(tf.transform),tf.header.frame_id))
    static_audit={k:dict(messages=len(v),distinct=len(set((s,p) for _,s,p in v)),parents=sorted(set(p for _,_,p in v))) for k,v in static_groups.items()}
    complete=Buffer(cache_time=Duration(seconds=300.))
    for _,tf in statics: complete.set_transform_static(tf,'recorded')
    for _,tf in sorted(dynamics,key=lambda p:ns(p[1])): complete.set_transform(tf,'recorded')
    chronological=Buffer(cache_time=Duration(seconds=10.))
    # Static transforms are latched and could predate rosbag startup. Use the
    # first recorded value per child initially, then apply updates in order.
    first=set()
    for _,tf in statics:
        if tf.child_frame_id not in first: chronological.set_transform_static(tf,'recorded'); first.add(tf.child_frame_id)
    timeline=sorted([(bt,tf,False) for bt,tf in dynamics]+[(bt,tf,True) for bt,tf in statics],key=lambda x:x[0]); index=0
    rows=[]
    node=types.SimpleNamespace(num_cameras=cfg['num_cameras'],global_frame=cfg['parent_frame'],cam_tf_frame=cfg['cam_frame'],lidar_tf_frame=cfg['lidar_frame'],camera_mask_max_skew_ns=int(cfg['camera_mask_max_skew_sec']*1e9),min_lidar_points=cfg['min_lidar_points'],sparse_lidar_points=cfg['sparse_lidar_points'])
    node.stamp_ns=Node.stamp_ns
    node.get_objmask_from_multiarray=types.MethodType(Node.get_objmask_from_multiarray,node)
    for bt,m in masks:
        stamp=ns(m)
        if stamp not in paired: continue
        received=paired[stamp]['received_ros_ns']
        while index<len(timeline) and timeline[index][0]<=received:
            _,tf,static=timeline[index]
            (chronological.set_transform_static if static else chronological.set_transform)(tf,'recorded');index+=1
        cloud=clouds.get(paired[stamp]['lidar_stamp_ns']); row=dict(stamp=stamp,received=received,actual=actual.get(stamp),query=m.query)
        if cloud is None: row['missing_cloud']=True;rows.append(row);continue
        node.active_query=' '.join(m.query.casefold().split())
        cam_data=Node.extract_data_from_obj_mask_msg(node,m)
        for name,buffer in [('complete',complete),('received_time',chronological)]:
            node.tf_buffer=buffer;node.localization_quality=None;node.triangulated_position=None;node.found_lidar_in_mask=False
            try:
                Node.check_lidar_in_mask(node,cam_data,cloud)
                row[name]=dict(found=node.found_lidar_in_mask,position=clean(node.triangulated_position),quality=clean(node.localization_quality))
            except Exception as e: row[name]=dict(error=type(e).__name__+':'+str(e))
        rows.append(row)
    db.close()
    counters={mode:dict(found=sum(r.get(mode,{}).get('found',False) for r in rows),errors=sum('error' in r.get(mode,{}) for r in rows),matched_positive=sum(bool(r.get(mode,{}).get('found') and r.get('actual',{} ) and np.linalg.norm(np.asarray(r[mode]['position'])-r['actual']['position'])<1e-6) for r in rows)) for mode in ('complete','received_time')}
    results.append(dict(trial=trial.name,static_audit=static_audit,counters=counters,rows=rows))
encoded=json.dumps(dict(node_module=Node.__module__,geometry_file=geometry.__file__,rounds=results),sort_keys=True).encode()
(OUT/'depth_node_contract.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=[{k:v for k,v in r.items() if k!='rows'} for r in results])))
