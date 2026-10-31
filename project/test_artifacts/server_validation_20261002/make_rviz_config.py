from pathlib import Path
import yaml

root=Path(__file__).resolve().parents[2]
data=yaml.safe_load((root/'tools/nizhen_wildos.rviz').read_text(encoding='utf8'))
manager=data['Visualization Manager']
manager['Global Options']['Frame Rate']=15
displays=manager['Displays']
displays[0]['Reference Frame']='spot1/base_link'
displays[0]['Alpha']=.18
displays[0]['Plane Cell Count']=50
manager['Views']['Current']['Distance']=38
manager['Views']['Current']['Pitch']=1.15
manager['Views']['Current']['Yaw']=0.0
for d in displays:
    if d['Name'] in ('TF','NavGraph Viz','Within Range Geofrontiers','Score Rings'):d['Enabled']=False
    if d['Name']=='WildOS Model Viz':d['Enabled']=False

def topic(name):
    return {'Value':name,'Depth':5,'Durability Policy':'Volatile','History Policy':'Keep Last','Reliability Policy':'Best Effort'}
for name,key,alpha in [('Global Costmap','global_costmap',.4),('Local Costmap','local_costmap',.6)]:
    displays.append({'Class':'rviz_default_plugins/Map','Name':name,'Enabled':True,
                     'Alpha':alpha,'Color Scheme':'costmap','Topic':topic('/wildos/viz/'+key)})
for name,key,color in [('Planned Vehicle Path','planned_path','255; 210; 60'),
                       ('GraphNav Route','graph_path','80; 150; 255'),
                       ('Executed Motion Section','received_plan','255; 80; 220'),
                       ('Vehicle Trail','../odom_trail','50; 230; 100')]:
    path='/wildos/odom_trail' if key.startswith('../') else '/wildos/viz/'+key
    displays.append({'Class':'rviz_default_plugins/Path','Name':name,'Enabled':True,
                     'Topic':topic(path),'Color':color,'Line Style':'Lines','Line Width':.1,
                     'Alpha':1,'Buffer Length':1})
displays.append({'Class':'rviz_default_plugins/Polygon','Name':'Vehicle Footprint','Enabled':True,
                 'Topic':topic('/wildos/viz/footprint'),'Color':'0; 255; 255','Alpha':1})
displays.append({'Class':'rviz_default_plugins/MarkerArray','Name':'Memory and Status','Enabled':True,
                 'Topic':topic('/wildos/debug_markers')})
data['Window Geometry']['Width']=1280; data['Window Geometry']['Height']=950
target=root/'nav2_adapter/config/beamng_navigation.rviz'
target.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True),encoding='utf8')
print(target)
