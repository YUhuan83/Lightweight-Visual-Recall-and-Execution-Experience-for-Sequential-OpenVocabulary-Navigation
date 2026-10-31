from pathlib import Path
import hashlib,json,re,sys,zipfile
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
label=sys.argv[1];assert re.fullmatch(r'baseline|candidate[0-9]*',label)
dest=OUT/label;dest.mkdir(exist_ok=False)
prior=json.loads((ROOT/'test_artifacts/recall_contract_repair_20261019/v2/candidate_delta.json').read_text())
names=set(prior['members'])
names.update('wildos_port/repro_check/src/nebula2-wildos/'+suffix for suffix in (
 'graphnav_mapper/src/mapper.cpp','graphnav_mapper/test/test_mapper.cpp',
 'graphnav_planner/src/planner.cpp','graphnav_planner/test/test_route_mechanism.cpp'))
names.add('wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/include/graphnav_planner/planner.hpp')
manifest=dict(baseline_bundle=prior['baseline_bundle'],members={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(names)},
              label=label,field_acceptance=False,production_deployment=False)
with zipfile.ZipFile(dest/'candidate_delta.zip','x',zipfile.ZIP_DEFLATED) as z:
 for p in names:z.write(ROOT/p,p)
 z.writestr('candidate_delta.json',json.dumps(manifest,indent=2)+'\n')
(dest/'candidate_delta.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
stage=(ROOT/'test_artifacts/goal_release_fence_20261019/stage_candidate.py').read_text(encoding='utf8')
(dest/'stage_candidate.py').write_text(stage,encoding='utf8',newline='\n')
(dest/'build_and_test.sh').write_text((OUT/'build_and_test.sh').read_text(encoding='utf8'),encoding='utf8',newline='\n')
print(json.dumps(dict(label=label,files=len(names),sha256=hashlib.sha256((dest/'candidate_delta.zip').read_bytes()).hexdigest())))
