import ast,difflib,hashlib,json,zipfile
from pathlib import Path
art=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
old=art/'execution_handoff_repair_20261025/candidate';out=art/'target_identity_repair_20261025';new=out/'candidate_r2'
base=json.loads((out/'base_hashes.json').read_text());v='wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
allowed={v+'/visual_navigation/explorfm_triangulation/localization_geometry.py':{'foreground_depth','_surface_quality','triangulate_bearings'},
 v+'/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py':{'ObjectMaskTriangulator.__init__','ObjectMaskTriangulator.on_query',
 'ObjectMaskTriangulator.process_observation','ObjectMaskTriangulator.check_lidar_in_mask'}}
def functions(path):
    out={}
    def walk(nodes,prefix=''):
        for n in nodes:
            if isinstance(n,ast.ClassDef):walk(n.body,prefix+n.name+'.')
            elif isinstance(n,ast.FunctionDef):out[prefix+n.name]=ast.dump(n,include_attributes=False)
    walk(ast.parse(path.read_text(encoding='utf8')).body);return out
changed={n for n,h in base.items() if hashlib.sha256((new/n).read_bytes()).hexdigest()!=h}
runtime={n for n in changed if '/test/' not in n};assert runtime==set(allowed),runtime
report={}
for n,expect in allowed.items():
    a,b=functions(old/n),functions(new/n)
    actual={k for k in a.keys()|b.keys() if a.get(k)!=b.get(k)}
    assert actual==expect,(n,actual)
    report[n]=dict(functions=sorted(actual),sha256=hashlib.sha256((new/n).read_bytes()).hexdigest())
(out/'production_scope_verified.json').write_text(json.dumps(report,indent=2))
before=json.loads((out/'localization_before.json').read_text());after=json.loads((out/'r2_localization_after.json').read_text())
assert len(before)==7 and len(after)==5
assert all(r['inside_current_mask'] and r['source']=='lidar' for r in after)
oldp=[r['position'][:2] for r in before[-3:]];newp=[r['position'][:2] for r in after[-3:]]
import numpy as np
span=lambda points:float(max(np.linalg.norm(np.array(a)-b) for a in points for b in points))
assert span(newp)<.2 and span(oldp)>2.5
summary=dict(before_proposals=7,after_proposals=5,before_last3_span_m=span(oldp),after_last3_span_m=span(newp),
    all_lidar_inside_mask=True,removed_uncertain_multiview=2,complete_search_passed=False)
(out/'recorded_replay_summary.json').write_text(json.dumps(summary,indent=2))
added=set(json.loads((out/'delta_receipt.json').read_text())['changed'])-set(base)
diff=[]
with zipfile.ZipFile(out/'v29_r2_delta_from_v28.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed}))
    for n in sorted(changed|added):
        z.write(new/n,n)
        if n.endswith('.py'):
            a=(old/n).read_text(encoding='utf8') if (old/n).exists() else ''
            b=(new/n).read_text(encoding='utf8')
            diff.extend(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='v28/'+n,tofile='v29_r2/'+n))
(out/'v28_to_v29_r2.diff').write_text(''.join(diff),encoding='utf8')
print(json.dumps(dict(scope=report,replay=summary),indent=2))
