import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'nav2_adapter'))
from wildos_nav2_adapter.graph_route import validate_vehicle_path
from wildos_nav2_adapter.motion_path import motion_sections
case=json.loads((root/'test_artifacts/server_validation_20261002/house_geometry_rviz_attempt2.json').read_text())
event=next(e for e in case['events'] if e['kind']=='planned_path' and e['phase']=='station_return' and e['poses']==25)
points,event_headings=event['points'],event['headings']
grid=NS(info=NS(width=400,height=400,resolution=.25,origin=NS(position=NS(x=690.,y=-70.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*160000)
def trace(frame,event,arg):
    if event=='return' and frame.f_code.co_name=='validate_vehicle_path' and arg=='excess_curvature':
        print('curvature failure',frame.f_lineno,{k:frame.f_locals.get(k) for k in ('i','j','k','quantized','turn','arc','bounded_turn','curvature','bin_angle')})
    return trace
sys.settrace(trace)
for first,last in [(0,len(points)-1),*motion_sections(points)]:
    p=points[first:last+1];h=event_headings[first:last+1]
    print(first,last,validate_vehicle_path(p,h,p,grid,p[-1],start=p[0],heading_quantization_bins=72))
sys.settrace(None)
out=root/'nav2_adapter/test/fixtures/town_analytic_join.json';out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps({'source':'2026-10-02 actual Smac station-return path; collision-free test isolates geometry',
                           'points':points,'headings':event_headings},indent=2),encoding='utf8')
