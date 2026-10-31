import sqlite3,glob,json
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
goal='/home/ly/execution_continuity_review_20261022/v17_costmap_unblock_retry/full/trial_20261023_190719/sessions/session_20261023_190719/goal_001/interfaces'
db=sqlite3.connect('file:'+glob.glob(goal+'/*.db3')[0]+'?mode=ro',uri=True)
def tid(n):
 r=db.execute('select id from topics where name=?',(n,)).fetchone();return r[0] if r else None
def strings(n):
 out=[]
 for bt,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid(n),)):
  m=deserialize_message(blob,String)
  try:d=json.loads(m.data)
  except:continue
  t=d.get('stamp_ns',bt)*1e-9;out.append((t,d))
 return out
def cmds():
 out=[]
 for bt,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid('/wildos/cmd_vel_stamped'),)):
  m=deserialize_message(blob,TwistStamped);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  out.append((t,float(m.twist.linear.x)))
 return out
plan=strings('/wildos/plan_status');ctrl=strings('/wildos/control_status');commands=cmds();out=[]
for a,b in zip(commands,commands[1:]):
 dt=b[0]-a[0]
 if dt<2:continue
 lo,hi=a[0]-1,b[0]+1
 ps=[];last=None
 for t,d in plan:
  if lo<=t<=hi:
   key=(d.get('status'),d.get('reason'),d.get('source'))
   if key!=last:
    ps.append({'t':round(t-a[0],2),'status':key[0],'reason':key[1],'source':key[2]});last=key
 cs=[];last=None
 for t,d in ctrl:
  if lo<=t<=hi:
   key=(d.get('reason'),d.get('controller_rejection'))
   if key!=last:
    cs.append({'t':round(t-a[0],2),'reason':key[0],'reject':key[1]});last=key
 out.append({'gap_s':round(dt,3),'from_s':round(a[0],3),'to_s':round(b[0],3),'v':[round(a[1],2),round(b[1],2)],'plans':ps,'control':cs})
print(json.dumps(out,indent=2))


