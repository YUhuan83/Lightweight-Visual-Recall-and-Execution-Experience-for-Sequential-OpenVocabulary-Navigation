from pathlib import Path
import hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
name='wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/include/graphnav_planner/planner.hpp'
source=ROOT/name;destination=OUT/'before'/name;assert not destination.exists()
destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,destination)
rows=json.loads((OUT/'before_manifest.json').read_text(encoding='utf8'))
rows.append(dict(path=name,sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
(OUT/'before_manifest.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf8')
print(rows[-1])
