"""Read-only end-to-end audit and live RViz evidence; no goal/control changes."""
from pathlib import Path
import json, os, sys, subprocess, sqlite3, time, hashlib, math
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
import rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import ListParameters,GetParameters
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
out=ROOT/'mechanism_audit'; out.mkdir(exist_ok=True)
rclpy.init(); n=Node('full_mechanism_read_only_audit')
report={'parameters':{},'endpoints':{},'processes':[],'images':{}}
def receive(key,m):
    a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(out/(key+'.png')),a)
    report['images'][key]={'width':m.width,'height':m.height,'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
for key,topic in [('model','/spot1/model_visualization'),('frontier_dashboard','/wildos/viz/frontier_traversability')]:
    n.create_subscription(Image,topic,lambda m,key=key:receive(key,m),qos_profile_sensor_data)
until=time.monotonic()+6
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
report['nodes']=[ns.rstrip('/')+'/'+name for name,ns in n.get_node_names_and_namespaces()]
for remote in ('/wildos_graph_path_bridge','/spot1/graphnav_mapper','/wildos_cmd_vel_to_control','/wildos_native_vehicle_control','/obj_mask_triangulator','/beamng_goal_standoff'):
    c=n.create_client(ListParameters,remote+'/list_parameters');g=n.create_client(GetParameters,remote+'/get_parameters')
    if not c.wait_for_service(timeout_sec=2):
        report['parameters'][remote]={'service_unavailable':True};continue
    f=c.call_async(ListParameters.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    if not f.done():report['parameters'][remote]={'timeout':True};continue
    names=f.result().result.names
    f=g.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    if not f.done():report['parameters'][remote]={'timeout':True};continue
    attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    values={}
    for key,v in zip(names,f.result().values):
        val=getattr(v,attrs[v.type],None) if v.type else None
        values[key]=list(val) if v.type>=5 else val
    report['parameters'][remote]=values
    n.destroy_client(c);n.destroy_client(g)
for topic in ('/odom','/spot1/odom','/wildos/cmd_vel','/spot1/traversal_feedback','/spot1/imgnav_waypoint','/spot1/imgnav_waypoint_raw','/spot1/object_mask','/spot1/scored_nav_graph','/wildos/viz/frontier_traversability'):
    def info(e):return {'node':e.node_namespace.rstrip('/')+'/'+e.node_name,'type':e.topic_type,'reliability':str(e.qos_profile.reliability),'durability':str(e.qos_profile.durability)}
    report['endpoints'][topic]={'publishers':[info(e) for e in n.get_publishers_info_by_topic(topic)],'subscribers':[info(e) for e in n.get_subscriptions_info_by_topic(topic)]}
for p in Path('/proc').glob('[0-9]*'):
    try:
        args=p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
        if any(('triangulat' in a or 'goal_standoff' in a or 'nav.py' in a) for a in args):
            report['processes'].append({'pid':p.name,'args':args,'state':p.joinpath('stat').read_text().split()[2]})
    except (OSError,UnicodeError):pass
n.destroy_node();rclpy.shutdown()
db=sqlite3.connect('file:'+str(ROOT/'body_collision_full_trial1/rosbag/rosbag_0.db3')+'?mode=ro',uri=True)
topics={name:(tid,typ) for tid,name,typ in db.execute('select id,name,type from topics')}
tid,typ=topics['/spot1/object_search/query'];cls=get_message(typ)
queries=[(ts,deserialize_message(blob,cls).data) for ts,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
house=next(ts for ts,q in queries if q=='house')
station=next(ts for ts,q in queries if q=='gas station' and ts>house)
end=db.execute('select max(timestamp) from messages').fetchone()[0]
phases={'station_observation':(queries[0][0],house),'house_search':(house,station),'station_return':(station,end+1)}
report['bag']={}
for phase,(start,stop) in phases.items():
    data={'duration':(stop-start)/1e9,'counts':{},'masks':[],'standoffs':[],'status_counts':{},'cmd':{}}
    for topic in ('/spot1/object_mask','/spot1/imgnav_waypoint','/wildos/cmd_vel','/wildos/planned_path','/wildos/full_planned_path','/spot1/model_visualization','/spot1/scored_nav_graph','/global_costmap/costmap','/lookahead_collision_arc'):
        tid,typ=topics[topic]
        data['counts'][topic]=db.execute('select count(*) from messages where topic_id=? and timestamp>=? and timestamp<?',(tid,start,stop)).fetchone()[0]
    for topic,key in [('/spot1/object_mask','masks'),('/spot1/imgnav_waypoint','standoffs')]:
        tid,typ=topics[topic]; cls=get_message(typ)
        for ts,blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<? order by timestamp',(tid,start,stop)):
            m=deserialize_message(blob,cls)
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            v={'elapsed':(ts-start)/1e9,'receive_delay':ts/1e9-stamp}
            if key=='masks':v.update(query=m.query,pixels=int(np.count_nonzero(np.asarray(m.object_mask.data))))
            else:v.update(position=[m.pose.position.x,m.pose.position.y,m.pose.position.z])
            data[key].append(v)
    tid,typ=topics['/wildos/plan_status'];cls=get_message(typ)
    for ts,blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<?',(tid,start,stop)):
        v=json.loads(deserialize_message(blob,cls).data)
        key=v['status']+':'+v.get('reason',v.get('detail',''))
        data['status_counts'][key]=data['status_counts'].get(key,0)+1
    tid,typ=topics['/wildos/cmd_vel'];cls=get_message(typ);cmds=[]
    for ts,blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<?',(tid,start,stop)):
        m=deserialize_message(blob,cls);cmds.append((ts,m.linear.x,m.angular.z))
    if cmds:
        vals=np.array(cmds)[:,1:]
        data['cmd']={'count':len(cmds),'nonzero':int((abs(vals[:,0])>.01).sum()),'max_linear':float(vals[:,0].max()),'min_linear':float(vals[:,0].min()),'max_angular':float(abs(vals[:,1]).max())}
    report['bag'][phase]=data
db.close()
(out/'mechanism_supplement.json').write_text(json.dumps(report,indent=2))
subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','x11grab','-video_size','1280x1024','-i',':99','-frames:v','1','-threads','1',str(out/'rviz_frontier.png')],check=True,timeout=15)
print(json.dumps({'out':str(out),'nodes':report['nodes'],'images':report['images'],'odom':report['endpoints']['/odom'],'feedback':report['endpoints']['/spot1/traversal_feedback'],'bag_summary':{k:{kk:vv for kk,vv in v.items() if kk not in ('masks','standoffs')} for k,v in report['bag'].items()}},indent=2),flush=True)

