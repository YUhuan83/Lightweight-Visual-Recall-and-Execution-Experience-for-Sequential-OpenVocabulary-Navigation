import ast,hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\arrival_decoupling_20261025')
allowed={
'nav2_adapter/wildos_nav2_adapter/arrival_criteria.py':{'vehicle_body_distance','within_body_arrival','arrival_status_matches'},
'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py':{'GraphPathBridge.__init__','GraphPathBridge.on_object_confirmation'},
'nav2_adapter/wildos_nav2_adapter/runtime_contract.py':{'navigation_runtime_contract'},
'official_adapter/beamng_wildos_adapter/goal_standoff.py':{'GoalStandoff.retry_confirmation'}}
def methods(text):
    found={}
    def walk(nodes,prefix=''):
        for n in nodes:
            if isinstance(n,ast.ClassDef):walk(n.body,prefix+n.name+'.')
            elif isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)):found[prefix+n.name]=ast.dump(n,include_attributes=False)
    walk(ast.parse(text).body);return found
out={}
with zipfile.ZipFile(root/'arrival_v25_source.zip') as base:
    for name,names in allowed.items():
        old=methods(base.read(name).decode('utf8'));new=methods((root/'candidate'/name).read_text(encoding='utf8'))
        changed={n for n in old.keys()|new.keys() if old.get(n)!=new.get(n)}
        assert changed==names,(name,changed,names)
        out[name]=dict(changed_functions=sorted(changed),sha256=hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest())
(root/'production_scope_verified.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(out,indent=2))
