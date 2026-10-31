import hashlib,json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
base=root.with_name('v71_heading_freshness')
manifest=json.loads((base/'full/frozen_source_manifest.json').read_text());changed=[]
for name,old in list(manifest['members'].items()):
    digest=hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()
    if digest!=old:changed.append(name)
    manifest['members'][name]=digest
(root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
library=root/'install/graphnav_planner/lib/libgraphnav_planner_component.so'
config=json.loads((root/'full/config.json').read_text())
config['library_sha256'][library.name]=hashlib.sha256(library.read_bytes()).hexdigest()
(root/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
for name in ('graph_path_bridge','continuous_graph_path_bridge','graph_route','arrival_criteria'):
    source=root/f'candidate/nav2_adapter/wildos_nav2_adapter/{name}.py'
    install=root/f'install/wildos_nav2_adapter/lib/python3.10/site-packages/wildos_nav2_adapter/{name}.py'
    assert source.read_bytes()==install.read_bytes(),name
scope=dict(baseline=str(base),changed=changed,arrival='visual XY distance strictly <3m; raw XYZ preserved',
    search_s=120,new_runtime_components=False,graphnav_library_sha256=config['library_sha256'][library.name],
    related_ros_tests=json.loads((root/'final_effective_regression.json').read_text())['tests'],
    native_tests=30,field_acceptance='pending',commit=None)
(root/'candidate_scope.json').write_text(json.dumps(scope,indent=2)+'\n');print(json.dumps(scope,indent=2))
