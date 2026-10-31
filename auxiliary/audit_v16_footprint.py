import sqlite3,glob,json,sys,math
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Odometry
sys.path.insert(0,'/home/ly/execution_continuity_review_20261022/v16_explore_during_reobserve/install/wildos_nav2_adapter/lib/python3.10/site-packages')
from wildos_nav2_adapter.graph_route import footprint_blockers,cell_cost
path='/home/ly/execution_continuity_review_20261022/v16_explore_during_reobserve/full/trial_20261023_184334/sessions/session_20261023_184334/goal_001/interfaces'
db=sqlite3.connect('file:'+glob.glob(path+'/*.db3')[0]+'?mode=ro',uri=True)
def tid(n):
 r=db.execute('select id from topics where name=?',(n,)).fetchone();return r[0] if r else None
def read(n,typ):
 out=[];i=tid(n)
 if i is None:return out
 for bt,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):
  out.append((bt*1e-9,deserialize_message(blob,typ)))
 return out
statuses=[]
for bt,m in read('/wildos/plan_status',String):
 try:d=json.loads(m.data)
 except:continue
 if d.get('status')=='start_footprint_blocked':
  statuses.append((d.get('stamp_ns',bt*1e9)*1e-9,d));break
print('event',json.dumps(statuses[0] if statuses else None))
if not statuses:raise SystemExit
start=statuses[0][0]
odom=read('/spot1/odom',Odometry)
def nearest_odom(t):return min(odom,key=lambda x:abs(x[0]-t))[1] if odom else None
for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
 maps=read(topic,OccupancyGrid); sampled=[];prevhash=None;lastsec=None
 for bt,g in maps:
  t=g.header.stamp.sec+g.header.stamp.nanosec*1e-9
  if not start-1<=t<=start+22:continue
  sec=int(t-start)
  if sec==lastsec:continue
  lastsec=sec
  o=nearest_odom(t)
  if o is None:continue
  p=o.pose.pose.position;q=o.pose.pose.orientation
  h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  center=(p.x,p.y);blocked=footprint_blockers(g,center,h)
  center_cost=cell_cost(g,*center);signature=hash(tuple(g.data))
  changed=prevhash is not None and signature!=prevhash
  sampled.append({'dt':round(t-start,2),'frame':g.header.frame_id,'res':g.info.resolution,'center':[round(p.x,2),round(p.y,2)],'yaw':round(h,2),'center_cost':center_cost,'blocked':bool(blocked),'blockers':len(blocked),'sample':blocked[:2],'map_changed':changed})
  prevhash=signature
 print(topic,json.dumps(sampled))
