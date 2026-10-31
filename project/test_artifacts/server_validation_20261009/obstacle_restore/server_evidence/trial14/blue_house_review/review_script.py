from pathlib import Path
import json,sqlite3,bisect,math
import numpy as np,rclpy
from rclpy.serialization import deserialize_message
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from object_search_msgs.msg import ObjectMaskWithTf
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation as R
from visual_navigation.utils.semantic_mask import largest_mask_component
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth
root=Path('/home/ly/town_memory_validation_20261008/B_green_house_station_trial14_obstacle_restore_20261009')
out=root/'blue_house_review';out.mkdir(exist_ok=True)
conn=sqlite3.connect('file:'+str(next((root/'rosbag').glob('*.db3')))+'?mode=ro',uri=True)
ids=dict(conn.execute('select name,id from topics'));s=lambda x:x.sec*10**9+x.nanosec
rclpy.init();tf=Buffer(cache_time=Duration(seconds=60))
a,b=1791513837000000000,1791513864000000000
for (blob,) in conn.execute('select data from messages where topic_id=?',(ids['/tf_static'],)):
 for t in deserialize_message(blob,TFMessage).transforms:tf.set_transform_static(t,'stopped_bag')
for (blob,) in conn.execute('select data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(ids['/tf'],a-5000000000,b+5000000000)):
 for t in deserialize_message(blob,TFMessage).transforms:tf.set_transform(t,'stopped_bag')
clouds=[]
for (blob,) in conn.execute('select data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(ids['/spot1/ouster/front/points_filtered'],a-2000000000,b+3000000000)):
 m=deserialize_message(blob,PointCloud2);clouds.append((s(m.header.stamp),m))
clouds.sort(key=lambda x:x[0]);times=[x[0] for x in clouds]
wanted=[1791513839202887205,1791513848479887646,1791513860035888195,1791513863084888339]
masks=[]
for (blob,) in conn.execute('select data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(ids['/spot1/object_mask'],a,b+3000000000)):
 m=deserialize_message(blob,ObjectMaskWithTf)
 if m.query=='green residential house':masks.append(m)
selected={s(min(masks,key=lambda m:abs(s(m.header.stamp)-w)).header.stamp) for w in wanted};rows=[]
for m in masks:
 stamp=s(m.header.stamp)
 if stamp not in selected:continue
 row={'mask_stamp_ns':stamp,'query':m.query}
 try:
  sizes=[d.size for d in m.object_mask.layout.dim];whole=np.asarray(m.object_mask.data,dtype=np.uint8).reshape(sizes)[:,0]>0;parts=[largest_mask_component(x) for x in whole];ci=int(np.argmax([x.sum() for x in parts]));mask=parts[ci]
  t=m.cam_transforms.transforms[ci].transform;rot=R.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix();trans=np.array([t.translation.x,t.translation.y,t.translation.z]);k=np.array(m.cam_infos[ci].k).reshape(3,3)
  j=bisect.bisect_left(times,stamp);j=min([i for i in (j-1,j) if 0<=i<len(times)],key=lambda i:abs(times[i]-stamp));cs,cloud=clouds[j]
  lt=tf.lookup_transform('spot1/odom',cloud.header.frame_id,Time(nanoseconds=cs)).transform;lr=R.from_quat([lt.rotation.x,lt.rotation.y,lt.rotation.z,lt.rotation.w]).as_matrix();ltrans=np.array([lt.translation.x,lt.translation.y,lt.translation.z])
  points=np.asarray([list(x) for x in point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True)],dtype=float).reshape(-1,3);world=points@lr.T+ltrans;camera=(world-trans)@rot; row.update(lidar_world_translation=ltrans.tolist(),lidar_world_rotation=lr.tolist())
  q=foreground_depth(camera,k,mask,20,8);yy,xx=np.where(mask)
  row.update(camera_position=trans.tolist(),camera_rotation=rot.tolist(),K=k.tolist(),mask_bbox=[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())],mask_pixels=int(mask.sum()),lidar_skew_sec=(cs-stamp)/1e9)
  oq=m.odom.pose.pose.orientation;op=m.odom.pose.pose.position;obr=R.from_quat([oq.x,oq.y,oq.z,oq.w]).as_matrix();ot=np.array([op.x,op.y,op.z])+obr@np.array([2.172,-.33,0.]);oc=((points@obr.T+ot)-trans)@rot;other=foreground_depth(oc,k,mask,20,8)
  row.update(mask_odom_lidar_translation=ot.tolist(),mask_odom_lidar_rotation=obr.tolist(),matched_pose_depth_world=(rot@other['position']+trans).tolist() if other else None,matched_pose_points=other['points'] if other else None,matched_pose_depth_camera=other['position'].tolist() if other else None)
  if q:
   p=rot@q['position']+trans;uv=k@q['position'];uv=uv[:2]/uv[2];pn=rot@q['plane_normal'];row.update(depth_world=p.tolist(),depth_camera=q['position'].tolist(),depth_projected_pixel=uv.tolist(),points=q['points'],confidence=q['confidence'],plane_normal=pn.tolist())
  valid=np.all(np.isfinite(camera),axis=1)&(camera[:,2]>.1);pts=camera[valid];wpts=world[valid];pix=pts@k.T;pix=pix[:,:2]/pix[:,2:];h,w=mask.shape;inside=(pix[:,0]>=0)&(pix[:,0]<w)&(pix[:,1]>=0)&(pix[:,1]<h);pix=pix[inside].astype(int);pts=pts[inside];wpts=wpts[inside];hits=mask[pix[:,1],pix[:,0]]
  row.update(mask_lidar_hits=int(hits.sum()),mask_depth_quantiles=np.quantile(pts[hits,2],[0,.25,.5,.75,1]).tolist() if hits.any() else [])
  np.savez_compressed(out/(str(stamp)+'_projection.npz'),pixels=pix,points_cam=pts,points_world=wpts,hits=hits,mask=mask,matched_pose_points_cam=oc,matched_pose_points_world=points@obr.T+ot)
 except Exception as e:row['error']=repr(e)
 rows.append(row)
conn.close();rclpy.shutdown();(out/'projection_review.json').write_text(json.dumps({'scope':'Read-only replay of stopped trial14 bag; no simulator movement or production change','rows':rows},indent=2));print(json.dumps(rows))
