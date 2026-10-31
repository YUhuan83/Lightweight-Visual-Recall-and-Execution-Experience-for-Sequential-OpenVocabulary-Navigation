from pathlib import Path
import hashlib, json, shutil
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
paths=[
 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/graph_observation_store.py',
 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py',
 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/execution_experience_store.py',
 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_visual_memory_map.py',
 'nav2_adapter/wildos_nav2_adapter/pointcloud.py',
]
rows=[]
for name in paths:
 p=ROOT/name; dest=OUT/'before'/name
 assert not dest.exists(), str(dest)
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
 rows.append(dict(path=name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(OUT/'before_manifest.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf8')
print('Preserved',len(rows),'source/test inputs')
