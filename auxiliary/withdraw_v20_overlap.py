"""Withdraw only this rejected experiment; retain all preceding dirty work."""
import ast
import hashlib
import json
import zipfile
from pathlib import Path

repo=Path(r'C:\Users\Yu\Desktop\拟真')
art=repo/'test_artifacts/fresh_command_retry_20261023'
snapshot=art/'v20_rejected_local'
snapshot.mkdir(exist_ok=False)
members=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
         'nav2_adapter/wildos_nav2_adapter/continuous_path.py',
         'nav2_adapter/test/test_motion_intent_handoff_ros.py',
         'nav2_adapter/test/test_committed_frontier_path_ros.py']
before={name:(repo/name).read_text(encoding='utf-8') for name in members}
for name in members:
    target=snapshot/name;target.parent.mkdir(parents=True,exist_ok=True)
    target.write_bytes((repo/name).read_bytes())

def method(text,name):
    start=text.index('    def '+name+'(')
    end=text.find('\n    def ',start+5)
    assert end!=-1
    return start,end,text[start:end]

with zipfile.ZipFile(art/'v20_delta.zip') as archive:
    frozen={name:archive.read(name).decode() for name in members}
after=dict(before)
name=members[0]
baseline=(art/'v19_base'/Path(name).name).read_text(encoding='utf-8')
source=after[name]
owner_gate=('        transport_issue = (self.execution_transport_issue(for_stop=True)\n'
            '                           if self.follow_quarantine or self.follow_release is not None\n'
            '                           else self.execution_transport_issue())')
for function in ('monitor_inputs','start_segment','dispatch_validated_path'):
    start,end,current=method(source,function)
    assert current.replace(owner_gate,'        transport_issue = self.execution_transport_issue()')==method(frozen[name],function)[2],function
    source=source[:start]+method(baseline,function)[2]+source[end:]
needle='    def reset(self, preserve_execution_corridor=False):\n        self.prepared_stop_plan = None\n'
assert source.count(needle)==1
after[name]=source.replace(needle,'    def reset(self, preserve_execution_corridor=False):\n')

name=members[1];source=after[name]
start=source.index('    def stop_plan_scope(');end=source.index('    def maybe_extend_path(')
assert source[start:end]==frozen[name][frozen[name].index('    def stop_plan_scope('):frozen[name].index('    def maybe_extend_path(')]
source=source[:start]+source[end:]
assert source.count('        self.prepared_stop_plan = None\n')==1
after[name]=source.replace('        self.prepared_stop_plan = None\n','')

name=members[2];source=after[name]
start=source.index('class StopPlanPreparationTests(');end=source.index('class MotionIntentHandoffTests(')
source=source[:start]+source[end:]
assert source.count('from std_msgs.msg import String\nimport rclpy\n')==1
after[name]=source.replace('from std_msgs.msg import String\nimport rclpy\n','from std_msgs.msg import String\n')

name=members[3];source=after[name]
start,end,current=method(source,'test_replacement_route_computes_while_execution_waits_for_stop_barrier')
assert current==method(frozen[name],'test_replacement_route_computes_while_execution_waits_for_stop_barrier')[2]
baseline=(art/'v19_base'/Path(name).name).read_text(encoding='utf-8')
after[name]=source[:start]+method(baseline,'test_replacement_route_waits_for_retired_action_stop_barrier')[2]+source[end:]

for name,source in after.items():
    ast.parse(source)
    assert 'prepared_stop_plan' not in source
    (repo/name).write_text(source,encoding='utf-8',newline='\n')
receipt=dict(reason='v20 A/B did not pass non-regression; source-generation correction withdrawn with experiment',
    archived=str(snapshot),members={name:dict(before=hashlib.sha256(before[name].encode()).hexdigest(),
        after=hashlib.sha256(after[name].encode()).hexdigest()) for name in members})
(art/'v20_withdrawal_receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
print(json.dumps(receipt,indent=2))
