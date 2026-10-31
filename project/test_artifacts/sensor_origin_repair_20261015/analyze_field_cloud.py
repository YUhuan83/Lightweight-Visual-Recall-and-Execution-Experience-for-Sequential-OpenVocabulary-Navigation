import json,sqlite3,math,collections
from pathlib import Path
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.duration import Duration
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from wildos_nav2_adapter.pointcloud import decode_xyz
from wildos_nav2_adapter.math3d import matrix_from_quaternion
s=Path('/home/ly/wildos-console/audit_results/session_20261015_203918')
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
def connect(folder):
 c=sqlite3.connect(next(folder.glob('*.db3')).as_uri()+'?mode=ro',uri=True)
 topics={n:(i,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')}
 return c,topics
c,t=connect(s/'goal_001/interfaces');sc,st=connect(s/'sensor_interfaces')
def prior(db,topics,name,ns):
 i,typ=topics[name];row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,ns)).fetchone()
 assert row,(name,ns)
 return row[0],deserialize_message(row[1],typ)
rclpy.init();buf=Buffer(cache_time=Duration(seconds=600))
ids=[st[n][0] for n in ('/tf','/tf_static')]
for ts,tid,blob in sc.execute('select timestamp,topic_id,data from messages where topic_id in (?,?) order by timestamp',ids):
 name='/tf' if tid==st['/tf'][0] else '/tf_static';msg=deserialize_message(blob,st[name][1])
 for tr in msg.transforms:
  if name=='/tf_static':buf.set_transform_static(tr,'sealed_bag')
  else:buf.set_transform(tr,'sealed_bag')
events=[json.loads(l) for l in (s/'goal_001/events.jsonl').read_text().splitlines()]
fail=[e for e in events if e.get('kind')=='plan_status' and e.get('status') in ('no_vehicle_path','follow_failed')]
reports=[]
for e in ([fail[0],fail[-1]] if fail else []):
 ns=e['stamp_ns'];mt,m=prior(c,t,'/global_costmap/costmap',ns);ot,o=prior(c,t,'/spot1/odom',ns)
 q=o.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));pos=o.pose.pose.position
 resolution=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y
 grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
 yy,xx=np.where(grid==100);wx=ox+(xx+.5)*resolution;wy=oy+(yy+.5)*resolution
 dx=wx-pos.x;dy=wy-pos.y;bx=dx*np.cos(yaw)+dy*np.sin(yaw);by=-dx*np.sin(yaw)+dy*np.cos(yaw)
 close=(bx>-2)&(bx<4.5)&(np.abs(by)<2)
 result={'event':e,'odom_frame':o.header.frame_id,'map_frame':m.header.frame_id,'map_receipt_age_sec':(ns-mt)/1e9,'odom_source_age_sec':(ns-o.header.stamp.sec*10**9-o.header.stamp.nanosec)/1e9,'near_lethal_cells':[{'world':[float(x),float(y)],'body':[float(a),float(b)]} for x,y,a,b in zip(wx[close],wy[close],bx[close],by[close])],'clouds':{}}
 for name in ('/wildos/points','/wildos/clearing_points','/spot1/ouster/front/points_filtered'):
  received,msg=prior(sc,st,name,ns);points=decode_xyz(msg).astype(float)
  try:
   tr=buf.lookup_transform(m.header.frame_id,msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
   qq=tr.rotation;rotation=np.asarray(matrix_from_quaternion([qq.x,qq.y,qq.z,qq.w]));v=tr.translation
   world=points@rotation.T+np.array([v.x,v.y,v.z]);dx=world[:,0]-pos.x;dy=world[:,1]-pos.y
   body=np.column_stack((dx*np.cos(yaw)+dy*np.sin(yaw),-dx*np.sin(yaw)+dy*np.cos(yaw),world[:,2]-pos.z))
   near=(body[:,0]>-2)&(body[:,0]<4.5)&(np.abs(body[:,1])<2)
   ij=np.floor((world[:,:2]-[ox,oy])/resolution).astype(int);inside=(ij[:,0]>=0)&(ij[:,0]<m.info.width)&(ij[:,1]>=0)&(ij[:,1]<m.info.height)
   costs=np.full(len(points),-999);costs[inside]=grid[ij[inside,1],ij[inside,0]];hits=near&(costs==100)
   result['clouds'][name]={'source_frame':msg.header.frame_id,'source_age_sec':(ns-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9,'total_points':len(points),'near_points':int(near.sum()),'near_points_on_lethal_cells':int(hits.sum()),'body_z_range':np.percentile(body[near,2],[0,50,100]).tolist() if near.any() else [],'near_lethal_point_examples':[{'source':points[i].tolist(),'world':world[i].tolist(),'body':body[i].tolist()} for i in np.flatnonzero(hits)[:30]]}
  except Exception as exc:result['clouds'][name]={'error':repr(exc),'source_frame':msg.header.frame_id}
 reports.append(result)
c.close();sc.close();rclpy.shutdown()
result={'scope':'Sealed nearest prior published cloud, odometry and occupancy grid; not private costmap or simulator ground truth','failure_events':len(fail),'reports':reports}
(s/'cloud_origin_review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
