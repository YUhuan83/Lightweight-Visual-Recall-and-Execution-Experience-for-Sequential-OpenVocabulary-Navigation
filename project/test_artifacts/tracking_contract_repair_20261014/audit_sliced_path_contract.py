"""Re-run full versus sliced admission on the identical recorded geometry."""
import json,math,sys
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'nav2_adapter'))
from wildos_nav2_adapter.graph_route import validate_vehicle_path
from wildos_nav2_adapter.motion_path import motion_sections
file=Path(__file__).parent/'green_house_dds_trial1/typed_interface_audit.json'
audit=json.loads(file.read_text());cases=[]
for rejection in audit['geometry_rejections']:
    for plan in rejection['original_plans']:
        points,headings=plan['points'],plan['headings']
        if len(points)<2:continue
        xmin=min(p[0] for p in points)-10;ymin=min(p[1] for p in points)-10
        width=math.ceil((max(p[0] for p in points)-xmin+10)/.25)
        height=math.ceil((max(p[1] for p in points)-ymin+10)/.25)
        grid=NS(info=NS(resolution=.25,width=width,height=height,
            origin=NS(position=NS(x=xmin,y=ymin),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*(width*height))
        whole=validate_vehicle_path(points,headings,points,grid,points[-1],
            start=points[0],minimum_turning_radius=8.,heading_quantization_bins=72)
        failures=[]
        for index,(a,b) in enumerate(motion_sections(points)):
            reason=validate_vehicle_path(points[a:b+1],headings[a:b+1],points,grid,points[b],
                start=points[a],minimum_turning_radius=8.,heading_quantization_bins=72)
            if reason:failures.append({'index':index,'first':a,'last':b,'reason':reason,
                'points':points[a:b+1],'headings':headings[a:b+1]})
        if whole is None and failures:
            cases.append({'event':rejection['event'],'plan':plan,'whole_reason':whole,'section_failures':failures})
out=file.with_name('sliced_path_counterexamples.json');out.write_text(json.dumps(cases,indent=2))
print(json.dumps({'cases':len(cases),'sections':[[f['index'] for f in c['section_failures']] for c in cases]}))
