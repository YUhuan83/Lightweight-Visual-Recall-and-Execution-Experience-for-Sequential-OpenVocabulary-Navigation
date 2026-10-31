from pathlib import Path
import hashlib,json,sys,zipfile
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
prior=json.loads((ROOT/'test_artifacts/goal_release_fence_20261019/candidate_delta.json').read_text())
names=set(prior['members'])
vis='wildos_port/repro_check/src/nebula2-wildos/visual_navigation/'
names.update(vis+'visual_navigation/wildos/'+n+'.py' for n in
             ('graph_observation_store','execution_experience_store','execution_scene_samples','search_evidence','nav'))
names.update(vis+'test/'+n+'.py' for n in ('test_directional_coverage_contract','test_execution_prior_contract','test_visual_memory_map','test_scoring_execution_contract'))
names.update(('nav2_adapter/wildos_nav2_adapter/pointcloud.py','nav2_adapter/test/test_pointcloud.py','tools/offline_navigation_check.py',
              'test_artifacts/passability_contract_20261019/run_ground_tests.py',
              'test_artifacts/recall_contract_repair_20261019/run_tests.py',
              'test_artifacts/recall_contract_repair_20261019/probe_decision_dds.py'))
manifest=dict(baseline_bundle=prior['baseline_bundle'],members={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(names)},
              field_acceptance=False,production_deployment=False)
destination=OUT if len(sys.argv)==1 else OUT/sys.argv[1]
if destination!=OUT:destination.mkdir(exist_ok=False)
with zipfile.ZipFile(destination/'candidate_delta.zip','x',zipfile.ZIP_DEFLATED) as z:
 for p in names:z.write(ROOT/p,p)
 z.writestr('candidate_delta.json',json.dumps(manifest,indent=2)+'\n')
(destination/'candidate_delta.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
stage=(ROOT/'test_artifacts/goal_release_fence_20261019/stage_candidate.py').read_text(encoding='utf8')
(destination/'stage_candidate.py').write_text(stage,encoding='utf8',newline='\n')
checks=(OUT/'run_checks.sh').read_text(encoding='utf8')
if destination!=OUT:checks=checks.replace('cd /home/ly/recall_contract_repair_20261019\n','cd /home/ly/recall_contract_repair_20261019/'+sys.argv[1]+'\n')
(destination/'run_checks.sh').write_text(checks,encoding='utf8',newline='\n') if destination!=OUT else None
print(json.dumps(dict(files=len(names),zip_sha256=hashlib.sha256((destination/'candidate_delta.zip').read_bytes()).hexdigest())))
