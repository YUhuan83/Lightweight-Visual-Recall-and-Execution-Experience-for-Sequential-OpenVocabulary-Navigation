"""Generate a separate, reviewable old-goal delivery counterexample; preserve joint probe."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
source=root/'probe_joint.py';body=source.read_text(encoding='utf8')
manifest=json.loads((root/'candidate_delta.json').read_text())
want=manifest['members']['test_artifacts/goal_release_repair_20261019/probe_joint.py']
assert hashlib.sha256(source.read_bytes()).hexdigest()==want
anchor="    bridge=GraphPathBridge();assert not bridge.follow_enabled\n"
assert body.count(anchor)==1
body=body.replace(anchor,"    observed_goals=[]\n    n.create_subscription(PoseStamped,'/spot1/graphnav_destination',lambda m:observed_goals.append(copy.deepcopy(m)),10)\n"+anchor)
anchor="    report['passed']=True\n";assert body.count(anchor)==1
body=body.replace(anchor,"""    assert observed_goals,'No actual goal sample'
    old_goal=observed_goals[0]
    report['old_goal_stamp_ns']=old_goal.header.stamp.sec*10**9+old_goal.header.stamp.nanosec
    bridge.waypoint_pub.publish(old_goal);pump(.2)
    late=update('late_old_goal_after_new_query')
    assert late['goal']==[40.,0.] and 'exploration_id' not in late,late
    report['gap_reproduced']=True
    report['scope']='Counterexample: actual old goal delivered after new-query clear; no repair claim.'
    report['passed']=True
""")
body=body.replace("'dds_'+time.strftime", "'late_goal_'+time.strftime")
target=root/'probe_late_goal.py';assert not target.exists();target.write_text(body,encoding='utf8',newline='\n')
print(json.dumps(dict(source_sha256=want,generated=str(target),sha256=hashlib.sha256(target.read_bytes()).hexdigest())))
