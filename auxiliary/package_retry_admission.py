import ast,difflib,hashlib,json,zipfile
from pathlib import Path
art=Path(__file__).parent.parent/'拟真/test_artifacts'
root=art/'retry_admission_repair_20261025';candidate=root/'candidate';reference=art/'confirmed_progress_repair_20261025/candidate'
base=json.loads((root/'base_hashes.json').read_text())
changed=[n for n,h in base.items() if hashlib.sha256((candidate/n).read_bytes()).hexdigest()!=h]
changed.append('nav2_adapter/test/test_continuous_retry_admission_ros.py')
runtime=[n for n in changed if '/test/' not in n]
assert runtime==['nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py'],runtime
name=runtime[0]
def methods(path):
    c=next(n for n in ast.parse(path.read_text(encoding='utf8')).body if isinstance(n,ast.ClassDef) and n.name=='ContinuousGraphPathBridge')
    return {n.name:ast.dump(n,include_attributes=False) for n in c.body if isinstance(n,ast.FunctionDef)}
before=methods(reference/name);after=methods(candidate/name)
assert {n for n in after if before.get(n)!=after[n]}=={'on_route'}
(root/'v31_to_v32.diff').write_text(''.join(difflib.unified_diff((reference/name).read_text(encoding='utf8').splitlines(True),(candidate/name).read_text(encoding='utf8').splitlines(True),fromfile='v31/'+name,tofile='v32/'+name)),encoding='utf8')
with zipfile.ZipFile(root/'retry_admission_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
report=dict(base='v31_confirmed_progress',runtime=runtime,method='ContinuousGraphPathBridge.on_route',changed={n:hashlib.sha256((candidate/n).read_bytes()).hexdigest() for n in changed})
(root/'source_scope.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
