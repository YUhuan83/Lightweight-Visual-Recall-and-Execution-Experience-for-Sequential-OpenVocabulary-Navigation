from pathlib import Path
import hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
paths=['graphnav_mapper/src/mapper.cpp','graphnav_mapper/test/test_mapper.cpp',
       'graphnav_planner/src/planner.cpp','graphnav_planner/test/test_route_mechanism.cpp']
rows=[]
for suffix in paths:
 name='wildos_port/repro_check/src/nebula2-wildos/'+suffix
 p=ROOT/name;dest=OUT/'before'/name;assert not dest.exists()
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
 rows.append(dict(path=name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(OUT/'before_manifest.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf8')
print('Preserved',len(rows),'C++ sources/tests')
