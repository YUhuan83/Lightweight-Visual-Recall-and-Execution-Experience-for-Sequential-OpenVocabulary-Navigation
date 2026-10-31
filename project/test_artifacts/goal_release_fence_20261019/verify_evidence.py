"""Check evidence/source identity; these checks do not establish vehicle acceptance."""
from pathlib import Path
import ast,hashlib,json,xml.etree.ElementTree as ET,zipfile
out=Path(__file__).resolve().parent;root=out.parents[1];prior=out.parent/'goal_release_repair_20261019'
manifest=json.loads((out/'candidate_delta.json').read_text())
baseline=root/'official_wildos_beamng_bundle.zip'
assert hashlib.sha256(baseline.read_bytes()).hexdigest()==manifest['baseline_bundle']
original=json.loads(baseline.with_suffix('.manifest.json').read_text())['members']
expected=dict(original);expected.update(manifest['members'])
for name,want in expected.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==want,name
with zipfile.ZipFile(out/'candidate_delta.zip') as z:
    assert z.testzip() is None
    for name,want in manifest['members'].items():assert hashlib.sha256(z.read(name)).hexdigest()==want,name
cpp=ET.parse(out/'server/cpp_results.xml').getroot()
assert int(cpp.attrib['tests'])==35 and int(cpp.attrib['failures'])==0 and int(cpp.attrib['errors'])==0
callbacks=json.loads((prior/'full_callback_report.json').read_text())
assert callbacks['passed'] and callbacks['tests']==570
assert sum(x['tests'] for x in callbacks['suites'])==570
ros=json.loads((out/'server/real_ros.json').read_text())
assert ros['passed'] and ros['tests']==60 and ros['failures']==ros['errors']==0
for name,identity in ros['source_identity'].items():
    assert identity['sha256']==manifest['members']['nav2_adapter/wildos_nav2_adapter/'+name+'.py']
runs=list((out/'server').glob('dds_*/result.json'));assert len(runs)==1
dds=json.loads(runs[0].read_text());assert dds['passed'] and not dds['errors']
assert not dds['complete_navigation_passed'] and not dds['beamng_started']
assert len(dds['stages'])==15
checks={stage['label']:stage['state'] for stage in dds['stages']}
for label in ('same_query_release_retains_backoff','release_after_explicit_goal_retains_backoff'):
    assert checks[label]['goal']==[40.,0.]
assert checks['old_goal_after_full_reset_is_ignored']['exploration_id']==checks['new_query_can_select_A_again']['exploration_id']
assert checks['old_goal_after_release_is_ignored']['exploration_id']==checks['release_reopens_exploration']['exploration_id']
assert checks['fresh_goal_after_full_reset_is_accepted']['goal']==[40.,0.]
assert checks['fresh_goal_after_invalid_future_is_accepted']['goal']==[40.,0.]
old=list((prior/'server').glob('late_goal_*/result.json'));assert len(old)==1
counter=json.loads(old[0].read_text());assert counter['gap_reproduced'] and not counter['errors']
assert counter['stages'][-1]['state']['goal']==[40.,0.]
assert 'exploration_id' not in counter['stages'][-1]['state']
sites=[]
for path in (root/'nav2_adapter/wildos_nav2_adapter').glob('*.py'):
    tree=ast.parse(path.read_text(encoding='utf8'))
    for definition in ast.walk(tree):
        if not isinstance(definition,(ast.FunctionDef,ast.AsyncFunctionDef)):continue
        for call in ast.walk(definition):
            if isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute) and call.func.attr=='request_graph_reset':
                full=any(k.arg=='reset_search' and isinstance(k.value,ast.Constant) and k.value.value is True for k in call.keywords)
                sites.append(dict(file=str(path.relative_to(root)),function=definition.name,line=call.lineno,
                    intent='reset_search' if full else 'release_or_retain_pending_full_reset'))
assert len(sites)==9,sites
assert [s['function'] for s in sites if s['intent']=='reset_search']==['on_query']
(out/'call_sites.json').write_text(json.dumps(sites,indent=2)+'\n',encoding='utf8')
summary=dict(evidence_consistent=True,baseline_members=len(original),source_members_verified=len(expected),
    cpp_tests=35,offline_callbacks=570,real_ros_callbacks=60,joint_dds_stages=15,
    old_goal_counterexample_preserved=True,call_sites=len(sites),vehicle_acceptance=False,
    runtime_planner_libraries=dds['runtime_identity']['libraries'])
(out/'evidence_review.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,indent=2))
