import csv, json, math, sys
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[4];sys.path.insert(0,str(ROOT/'nav2_adapter'))
from wildos_nav2_adapter.graph_route import validate_vehicle_path
from wildos_nav2_adapter.motion_path import motion_sections

report=[]
for file in sorted(Path(__file__).parent.glob('case*.csv')):
    rows=list(csv.DictReader(file.open())); points=[(float(r['x']),float(r['y'])) for r in rows];heads=[float(r['yaw']) for r in rows]
    xmin=min(p[0] for p in points)-10;ymin=min(p[1] for p in points)-10
    w=math.ceil((max(p[0] for p in points)-xmin+10)/.25);h=math.ceil((max(p[1] for p in points)-ymin+10)/.25)
    grid=NS(info=NS(resolution=.25,width=w,height=h,origin=NS(position=NS(x=xmin,y=ymin),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*(w*h))
    def check(a,b):return validate_vehicle_path(points[a:b+1],heads[a:b+1],points,grid,points[b],start=points[a],minimum_turning_radius=8.,heading_quantization_bins=72)
    sections=motion_sections(points)
    row={'file':file.name,'poses':len(points),'whole':check(0,len(points)-1),
         'sections':[{'first':a,'last':b,'reason':check(a,b)} for a,b in sections]}
    report.append(row)
Path(__file__).with_name('output_analysis.json').write_text(json.dumps(report,indent=2))
for r in report: print(json.dumps(r))
