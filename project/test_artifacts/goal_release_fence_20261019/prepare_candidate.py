"""Preserve first candidate; generate the extended DDS regression and freeze its successor."""
from pathlib import Path
import hashlib,json,shutil,zipfile
out=Path(__file__).resolve().parent;root=out.parents[1]
prior=out.parent/'goal_release_repair_20261019'
original=json.loads((prior/'candidate_delta.json').read_text())
source=prior/'probe_joint.py'
assert hashlib.sha256(source.read_bytes()).hexdigest()==original['members']['test_artifacts/goal_release_repair_20261019/probe_joint.py']
body=source.read_text(encoding='utf8')
anchor="    bridge=GraphPathBridge();assert not bridge.follow_enabled\n";assert body.count(anchor)==1
body=body.replace(anchor,"    observed_goals=[]\n    n.create_subscription(PoseStamped,'/spot1/graphnav_destination',lambda m:observed_goals.append(copy.deepcopy(m)),10)\n"+anchor)
anchor="    report['passed']=True\n";assert body.count(anchor)==1
body=body.replace(anchor,"""    assert observed_goals,'No actual goal sample'
    old_goal=observed_goals[0]
    bridge.waypoint_pub.publish(old_goal);pump(.2)
    late=update('old_goal_after_full_reset_is_ignored')
    assert late['goal']==[20.,0.] and late['exploration_id']==f['exploration_id'],late
    bridge.relay_destination(goal);pump(.2)
    fresh_b=update('fresh_goal_after_full_reset_is_accepted')
    assert fresh_b['goal']==[40.,0.] and 'exploration_id' not in fresh_b,fresh_b
    older_b=copy.deepcopy(observed_goals[-1])
    newer_a=copy.deepcopy(goal);newer_a.pose.position.x=20.
    bridge.relay_destination(newer_a);pump(.2)
    accepted_a=update('newer_explicit_goal_is_accepted')
    assert accepted_a['goal']==[20.,0.] and 'exploration_id' not in accepted_a,accepted_a
    bridge.waypoint_pub.publish(older_b);pump(.2)
    reordered=update('older_goal_in_same_phase_is_ignored')
    assert reordered['goal']==[20.,0.],reordered
    future_goal=copy.deepcopy(older_b);future_goal.header.stamp=n.get_clock().now().to_msg()
    future_goal.header.stamp.sec+=3600
    bridge.waypoint_pub.publish(future_goal);pump(.2)
    future_state=update('future_goal_is_ignored')
    assert future_state['goal']==[20.,0.],future_state
    bridge.relay_destination(goal);pump(.2)
    after_invalid=update('fresh_goal_after_invalid_future_is_accepted')
    assert after_invalid['goal']==[40.,0.] and 'exploration_id' not in after_invalid,after_invalid
    accepted_b=copy.deepcopy(observed_goals[-1])
    semantic_release()
    released=update('release_reopens_exploration')
    assert released['goal']==[20.,0.] and released['exploration_id'],released
    bridge.waypoint_pub.publish(accepted_b);pump(.2)
    late_release=update('old_goal_after_release_is_ignored')
    assert late_release['exploration_id']==released['exploration_id'],late_release
    report['full_elapsed_since_refusal_sec']=time.monotonic()-refused_at
    report['passed']=True
""")
target=out/'probe_joint.py';assert not target.exists();target.write_text(body,encoding='utf8',newline='\n')
files=list(original['members'])
files+=['wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/include/graphnav_planner/destination_fence.hpp',
        'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/test/test_route_mechanism.cpp',
        'test_artifacts/goal_release_fence_20261019/probe_joint.py']
manifest=dict(baseline_bundle=original['baseline_bundle'],members={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},
    production_deployment=False,field_acceptance=False)
with zipfile.ZipFile(out/'candidate_delta.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(root/p,p)
    z.writestr('candidate_delta.json',json.dumps(manifest,indent=2)+'\n')
(out/'candidate_delta.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
shutil.copyfile(prior/'stage_candidate.py',out/'stage_candidate.py')
body=(prior/'build_candidate.sh').read_text(encoding='utf8').replace('/home/ly/goal_release_repair_20261019','/home/ly/goal_release_fence_20261019')
(out/'build_candidate.sh').write_text(body,encoding='utf8',newline='\n')
body=(prior/'run_checks.sh').read_text(encoding='utf8').replace('/home/ly/goal_release_repair_20261019','/home/ly/goal_release_fence_20261019')
body=body.replace('candidate/test_artifacts/goal_release_repair_20261019/probe_joint.py','candidate/test_artifacts/goal_release_fence_20261019/probe_joint.py')
(out/'run_checks.sh').write_text(body,encoding='utf8',newline='\n')
print(json.dumps(dict(members=len(files),source_sha256=manifest['members']['wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner_node.cpp']),indent=2))
