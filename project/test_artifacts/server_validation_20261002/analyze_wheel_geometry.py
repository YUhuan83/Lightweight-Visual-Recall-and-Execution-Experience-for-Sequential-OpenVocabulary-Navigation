import json
import math
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
data=json.loads((root/'wheel_geometry_probe.json').read_text())
lua=json.loads(data['lua'])
f=np.asarray(data['state']['dir']); f/=np.linalg.norm(f)
up=np.asarray(data['state']['up']); left=np.cross(up,f); left/=np.linalg.norm(left)
up=np.cross(f,left); rotation=np.column_stack((f,left,up))
wheels={}
for w in lua['wheels']:
    vectors=[np.asarray([w[str(n)][k] for k in ('x','y','z')]) for n in (w['node1'],w['node2'])]
    wheels[w['name']]=(np.mean(vectors,axis=0)@rotation).tolist()
front=(np.asarray(wheels['FL'])+wheels['FR'])/2
rear=(np.asarray(wheels['RL'])+wheels['RR'])/2
pos=np.asarray(data['state']['pos'])
bbox=(np.asarray(list(data['bbox'].values()))-pos)@rotation
report={'wheel_centers_body':wheels,'front_axle_body':front.tolist(),
        'rear_axle_body':rear.tolist(),'wheelbase':float(front[0]-rear[0]),
        'body_extents_from_reference':{'x':[float(bbox[:,0].min()),float(bbox[:,0].max())],
                                      'y':[float(bbox[:,1].min()),float(bbox[:,1].max())]},
        'body_extents_from_rear_axle':{'x':[float(bbox[:,0].min()-rear[0]),float(bbox[:,0].max()-rear[0])],
                                      'y':[float(bbox[:,1].min()-rear[1]),float(bbox[:,1].max()-rear[1])]}}
samples=[json.loads(line) for line in (root/'server_evidence/house_attempt5/native_samples.jsonl').read_text().splitlines()]
curves=[]
def h(s):return math.atan2(s['direction'][1],s['direction'][0])
def rear_xy(s):
    heading=h(s)
    return [s['position'][0]+math.cos(heading)*rear[0]-math.sin(heading)*rear[1],
            s['position'][1]+math.sin(heading)*rear[0]+math.cos(heading)*rear[1]]
for i,a in enumerate(samples):
    steer=a.get('actual',{}).get('steering_input')
    if steer is None or abs(steer)<.59 or a['speed']<.3:continue
    for b in samples[i+1:]:
        current=b.get('actual',{}).get('steering_input')
        if current is None or abs(current-steer)>.015 or b['speed']<.3 or b['sent']['gear']!=a['sent']['gear']:break
        gap=math.dist(rear_xy(a),rear_xy(b))
        if gap>=1:
            angle=abs(math.atan2(math.sin(h(b)-h(a)),math.cos(h(b)-h(a))))
            if angle>.01:
                radius=gap/(2*math.sin(angle/2))
                curves.append({'radius':radius,'angle_per_input':math.atan(report['wheelbase']/radius)/abs(steer)})
            break
report['measured_saturated_rear_radius_median']=float(np.median([c['radius'] for c in curves]))
report['estimated_steering_angle_per_input']=float(np.median([c['angle_per_input'] for c in curves]))
report['saturated_samples']=len(curves)
(root/'wheel_geometry_analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
