"""Read-only mechanism audit from live parameters and the completed test bag."""
from pathlib import Path
import json, os, sys, subprocess, sqlite3, zipfile, hashlib, time
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, '--runtime-child'], env=runtime_env()).returncode)
out=ROOT/'mechanism_audit'
out.mkdir(exist_ok=True)
report={'processes':[], 'parameters':{}, 'nodes':[], 'bag':{}}
for name in ('vision','graph_bridge','planner','controller','visualization','official_adapter','mapper','cloud_filter','speed_control','native_control','graphnav_planner','rviz'):
    file=ROOT/(name+'.pid')
    if not file.exists(): continue
    pid=file.read_text().strip()
    proc=Path('/proc',pid)
    if not proc.exists(): continue
    args=proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
    report['processes'].append(dict(name=name,pid=pid,args=args,state=proc.joinpath('stat').read_text().split()[2]))
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters
rclpy.init()
n=Node('read_only_mechanism_audit')
time.sleep(1)
report['nodes']=[ns.rstrip('/')+'/'+name for name,ns in n.get_node_names_and_namespaces()]
for remote in ('/wildos_graph_path_bridge','/planner_server','/controller_server','/global_costmap/global_costmap','/local_costmap/local_costmap','/wildos_official_cloud_filter','/wildos_cmd_vel_to_control','/spot1/graphnav_planner','/beamng_wildos_adapter'):
    c=n.create_client(ListParameters,remote+'/list_parameters')
    g=n.create_client(GetParameters,remote+'/get_parameters')
    if not c.wait_for_service(timeout_sec=1):
        report['parameters'][remote]={'missing':True}; continue
    f=c.call_async(ListParameters.Request())
    rclpy.spin_until_future_complete(n,f,timeout_sec=4)
    if not f.done(): report['parameters'][remote]={'timeout':True};continue
    names=f.result().result.names
    f=g.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(n,f,timeout_sec=4)
    if not f.done(): report['parameters'][remote]={'timeout':True};continue
    values={}
    attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    for key,v in zip(names,f.result().values):
        val=getattr(v,attrs[v.type],None) if v.type else None
        if v.type>=5: val=list(val)
        values[key]=val
    report['parameters'][remote]=values
    print('parameters',remote,len(values),flush=True)
    n.destroy_client(c);n.destroy_client(g)
n.destroy_node();rclpy.shutdown()
print('Reading recorded bag',flush=True)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+str(ROOT/'body_collision_full_trial1/rosbag/rosbag_0.db3')+'?mode=ro',uri=True)
topics={name:(tid,typ) for tid,name,typ in db.execute('select id,name,type from topics')}
report['bag']['topics']={name:{'id':v[0],'type':v[1]} for name,v in topics.items()}
querytopic=topics['/spot1/object_search/query']
queries=[dict(timestamp=t,query=deserialize_message(blob,get_message(querytopic[1])).data) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(querytopic[0],))]
report['bag']['queries']=queries
switched=next(v['timestamp'] for v in queries if v['query']=='gas station' and v['timestamp']>queries[0]['timestamp']+30e9)
report['bag']['station_switch_timestamp']=switched
for topic in ('/spot1/remembered_object_feedback','/wildos/memory_selection','/wildos/full_planned_path','/spot1/graphnav_planner/path','/spot1/imgnav_waypoint','/spot1/remembered_object_candidates'):
    if topic not in topics:continue
    tid,typ=topics[topic];cls=get_message(typ)
    values=[]
    for ts,blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? order by timestamp',(tid,switched)):
        m=deserialize_message(blob,cls)
        entry={'timestamp':ts,'after_switch':(ts-switched)/1e9}
        if hasattr(m,'data'):
            try:entry['value']=json.loads(m.data)
            except ValueError:entry['value']=m.data
        elif hasattr(m,'poses'):
            import math
            entry['frame']=m.header.frame_id
            entry['points']=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
            entry['headings']=[math.atan2(2*(p.pose.orientation.w*p.pose.orientation.z+p.pose.orientation.x*p.pose.orientation.y),1-2*(p.pose.orientation.y**2+p.pose.orientation.z**2)) for p in m.poses]
        elif hasattr(m,'pose'):
            entry['frame']=m.header.frame_id
            entry['position']=[m.pose.position.x,m.pose.position.y,m.pose.position.z]
        values.append(entry)
    report['bag'][topic]=values
print('feedback',report['bag'].get('/spot1/remembered_object_feedback'),flush=True)
print('evaluated paths',len(report['bag'].get('/wildos/full_planned_path',[])),flush=True)
db.close()
sourcefiles=[]
for base in (WS/'src/wildos_nav2_adapter',WS/'src/beamng_wildos_adapter',WS/'src/nebula2-wildos/visual_navigation',WS/'src/nebula2-wildos/graphnav',Path('/home/ly/pointgoal_kit')):
    if not base.exists():continue
    for file in base.rglob('*'):
        if file.is_file() and file.suffix in ('.py','.yaml','.yml','.rviz','.cpp','.hpp') and '__pycache__' not in file.parts:
            sourcefiles.append(file)
with zipfile.ZipFile(out/'runtime_sources.zip','w',zipfile.ZIP_DEFLATED) as z:
    for file in sourcefiles:z.write(file,str(file).lstrip('/'))
report['source_hashes']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sourcefiles}
(out/'mechanism.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'out':str(out),'source_count':len(sourcefiles),'processes':report['processes'],'nodes':report['nodes']},indent=2),flush=True)

