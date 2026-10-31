import ast,hashlib,json
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');art=repo/'test_artifacts'
old=art/'arrival_decoupling_20261025/candidate';out=art/'execution_handoff_repair_20261025';new=out/'candidate'
allowed={
 'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py':{
 'GraphPathBridge.extension_candidates','GraphPathBridge.start_segment','GraphPathBridge.maybe_side_lookback'},
 'nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py':{'ContinuousGraphPathBridge.extension_candidates'},
 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py':{
 'ObjectMaskTriangulator.process_observation','ObjectMaskTriangulator.bearing_observation'}}
def methods(path):
    tree=ast.parse(path.read_text(encoding='utf8'));result={}
    for node in tree.body:
        if isinstance(node,ast.ClassDef):
            for f in node.body:
                if isinstance(f,ast.FunctionDef):result[node.name+'.'+f.name]=ast.dump(f,include_attributes=False)
    return result
base=json.loads((out/'base_hashes.json').read_text());changed={}
for n in base:
    if (old/n).exists() and hashlib.sha256((old/n).read_bytes()).hexdigest()!=hashlib.sha256((new/n).read_bytes()).hexdigest():
        changed[n]=hashlib.sha256((new/n).read_bytes()).hexdigest()
runtime={n for n in changed if '/test/' not in n}
assert runtime==set(allowed),(runtime,allowed)
scope={}
for n,expected in allowed.items():
    a,b=methods(old/n),methods(new/n)
    actual={k for k in a.keys()|b.keys() if a.get(k)!=b.get(k)}
    assert actual==expected,(n,actual)
    scope[n]=dict(changed_functions=sorted(actual),sha256=changed[n])
report=dict(runtime_scope=scope,changed_members=changed,
    new_test='wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_fresh_bearing.py')
(out/'production_scope_verified.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
