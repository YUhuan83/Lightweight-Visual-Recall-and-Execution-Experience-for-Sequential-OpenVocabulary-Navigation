"""Align observed DDS commands with the latest received odometry/route."""
import collections,json,math,sys
from pathlib import Path
trial=int(sys.argv[1]);root=Path(__file__).parent
path=root/f'physical_cusp_component{trial}_observer/messages.jsonl'
counts=collections.Counter();odom=native=control=plan=None;rows=[];states=[]
for line in path.read_text().splitlines():
    r=json.loads(line);topic=r['topic'];counts[topic]+=1;m=r['message'];now=r['received_ros_ns']/1e9
    if topic=='/spot1/odom':odom=m
    elif topic=='/wildos/native_control_status':native=json.loads(m['data'])
    elif topic=='/wildos/control_status':control=json.loads(m['data'])
    elif topic=='/wildos/planned_path':plan=m
    elif topic=='/wildos/plan_status':states.append(json.loads(m['data']))
    elif topic=='/wildos/cmd_vel' and odom and plan and plan['poses']:
        p=odom['pose']['pose'];q=p['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
        end=plan['poses'][-1]['pose'];x,y=p['position']['x'],p['position']['y'];dx=end['position']['x']-x;dy=end['position']['y']-y
        h=odom['header']['stamp'];age=now-h['sec']-h['nanosec']/1e9
        rows.append(dict(time=now,pose=[x,y,yaw],endpoint=end,gap=math.hypot(dx,dy),
            body_endpoint=[dx*math.cos(yaw)+dy*math.sin(yaw),-dx*math.sin(yaw)+dy*math.cos(yaw)],
            odom_age=age,cmd=m,odom_velocity=odom['twist']['twist'],native=native,control=control))
near=[r for r in rows if r['gap']<.3]
summary={'scope':'Received DDS alignment; not exact controller tick pose',
    'topic_counts':dict(counts),'states':states,'near_endpoint_commands':near,'commands':rows}
out=root/f'physical_cusp_component{trial}_observer/alignment.json'
out.write_text(json.dumps(summary,indent=2))
print(json.dumps({'topic_counts':dict(counts),'commands':len(rows),'near_endpoint':len(near),
    'moving_commands_inside_disk':sum(r['gap']<=.2 and abs(r['cmd']['linear']['x'])>.01 for r in near),
    'zero_commands_inside_disk':sum(r['gap']<=.2 and abs(r['cmd']['linear']['x'])<=.01 for r in near),
    'statuses':[s['status'] for s in states]},indent=2))
for row in near[-14:]:
    print(json.dumps({k:v for k,v in row.items() if k not in ('native','control','endpoint')}))
