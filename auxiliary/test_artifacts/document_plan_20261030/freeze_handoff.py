import hashlib,json,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v74_document_handoff')
base=root.with_name('v71_heading_freshness');previous=root.with_name('v73_document_plan')
manifest=json.loads((base/'full/frozen_source_manifest.json').read_text());changed=[];incremental=[]
for name,old in list(manifest['members'].items()):
    content=(root/'candidate'/name).read_bytes();digest=hashlib.sha256(content).hexdigest()
    if digest!=old:changed.append(name)
    if content!=(previous/'candidate'/name).read_bytes():incremental.append(name)
    manifest['members'][name]=digest
(root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
for name in ('graph_path_bridge','continuous_graph_path_bridge','graph_route','arrival_criteria'):
    assert (root/f'candidate/nav2_adapter/wildos_nav2_adapter/{name}.py').read_bytes()==(root/f'install/wildos_nav2_adapter/lib/python3.10/site-packages/wildos_nav2_adapter/{name}.py').read_bytes(),name
library=previous/'install/graphnav_planner/lib/libgraphnav_planner_component.so'
digest=hashlib.sha256(library.read_bytes()).hexdigest()
config=json.loads((root/'full/config.json').read_text());config['library_sha256'][library.name]=digest
(root/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
regression=json.loads((root/'isolated_ros_report.json').read_text());assert all(r['passed'] for r in regression)
scope=dict(baseline=str(base),previous=str(previous),changed=changed,incremental_changed=incremental,
    arrival='visual XY distance strictly <3m; raw XYZ retained',new_runtime_components=False,search_s=120,
    related_ros_tests=sum(r['tests'] for r in regression),graphnav_library_sha256=digest,
    graphnav_library_source=str(library),field_acceptance='pending',commit=None)
(root/'candidate_scope.json').write_text(json.dumps(scope,indent=2)+'\n')
with zipfile.ZipFile(root/'review_changes.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name in changed:z.write(root/'candidate'/name,name)
print(json.dumps(scope,indent=2))
