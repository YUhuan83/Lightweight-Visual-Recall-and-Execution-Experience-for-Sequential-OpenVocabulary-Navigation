"""Trace the exact existing validator rejection; no production behavior changes."""
import json, math, sys, importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'nav2_adapter'))
spec = importlib.util.spec_from_file_location('wildos_nav2_adapter.graph_route_before',
    Path(__file__).parent/'before/graph_route_dds06bb.py')
module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
validate_vehicle_path = module.validate_vehicle_path
from wildos_nav2_adapter.motion_path import motion_sections

def traced(points, headings, route, grid):
    records = []
    def trace(frame, event, arg):
        if frame.f_code is validate_vehicle_path.__code__ and event == 'return':
            local = frame.f_locals
            record = {'reason': arg, 'line': frame.f_lineno}
            if arg == 'excess_curvature':
                for name in ('i', 'j', 'k', 'arc', 'turn', 'bounded_turn', 'uncertainty', 'curvature'):
                    if name in local: record[name] = local[name]
                for name in ('traveled', 'checked_headings', 'quantized_headings'):
                    record[name] = local.get(name)
            records.append(record)
        return trace
    sys.settrace(trace)
    try:
        validate_vehicle_path(points, headings, route, grid, points[-1], start=points[0],
                              minimum_turning_radius=8., heading_quantization_bins=72)
    finally:
        sys.settrace(None)
    return records[-1]

cases = json.loads((Path(__file__).parent / 'green_house_dds_trial1/sliced_path_counterexamples.json').read_text())
results = []
for case in cases:
    plan = case['plan']; points, headings = plan['points'], plan['headings']
    xmin = min(p[0] for p in points)-10; ymin = min(p[1] for p in points)-10
    width = math.ceil((max(p[0] for p in points)-xmin+10)/.25)
    height = math.ceil((max(p[1] for p in points)-ymin+10)/.25)
    grid = NS(info=NS(resolution=.25, width=width, height=height,
        origin=NS(position=NS(x=xmin,y=ymin), orientation=NS(x=0.,y=0.,z=0.,w=1.))), data=[0]*(width*height))
    entry = {'event': case['event'], 'whole': traced(points, headings, points, grid), 'sections': []}
    for a,b in motion_sections(points):
        trace = traced(points[a:b+1],headings[a:b+1],points,grid)
        if trace['reason']:
            trace['first'] = a; trace['last'] = b
            i, j = trace['i'], trace['j']
            trace['window'] = [{'index': n+a,'point':points[n+a],'heading':headings[n+a],
                'checked':trace['checked_headings'][n], 'quantized':trace['quantized_headings'][n]}
                for n in range(max(0,i-1),min(b-a+1,j+2))]
            trace.pop('traveled'); trace.pop('checked_headings'); trace.pop('quantized_headings')
            entry['sections'].append(trace)
    results.append(entry)
out = Path(__file__).parent / 'green_house_dds_trial1/curvature_window_trace.json'
out.write_text(json.dumps(results,indent=2))
for e in results:
    print(json.dumps({'event':e['event'],'whole':e['whole'], 'sections':e['sections']}))
