"""Add input identities to the frozen v19 diagnostics, without motion changes."""
import ast
import hashlib
import json
import textwrap
import zipfile
from pathlib import Path

repo=Path(r'C:\Users\Yu\Desktop\拟真')
art=repo/'test_artifacts/fresh_command_retry_20261023'
base=art/'v19_base'
members=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
         'nav2_adapter/test/test_graph_path_bridge_routing.py']
old=(base/'graph_path_bridge.py').read_text(encoding='utf-8')
new=(repo/members[0]).read_text(encoding='utf-8')
def method(text,name):
    start=text.index('    def '+name+'(');end=text.find('\n    def ',start+5)
    assert end!=-1
    return start,end,text[start:end]
for name in ('status','try_candidate','send_motion_segment'):
    start,end,body=method(old,name)
    if name=='status':
        first=new.index('                            "replay_inputs": {')
        last=new.index('                            **details}',first)
        body=body.replace('                            **details}',new[first:last]+'                            **details}')
    elif name=='try_candidate':
        body=body.replace("                    background=bool(self.following))",
            "                    background=bool(self.following),\n"
            "                    requested_goal_pose=[float(x),float(y),float(heading)],\n"
            "                    requested_start_pose=([*xy(goal.start.pose),yaw(goal.start.pose)]\n"
            "                                          if goal.use_start else None),\n"
            "                    bridge_odom_pose=[*xy(self.odom.pose.pose),yaw(self.odom.pose.pose)],\n"
            "                    planner_use_start=bool(goal.use_start))")
    else:
        body=body.replace('                            position_error=offset, heading_error=angle,\n',
            '                            position_error=offset, heading_error=angle,\n'
            '                            path_source_stamp_ns=stamp_ns(path),\n'
            '                            rejected_section_goal=list(target),\n')
        body=body.replace('        path.header.stamp = self.get_clock().now().to_msg()',
            '        source_path_stamp_ns=stamp_ns(path)\n        path.header.stamp = self.get_clock().now().to_msg()')
        marker='                    actual_start_stamp_ns=(self.odom.header.stamp.sec*10**9+self.odom.header.stamp.nanosec),\n'
        body=body.replace(marker,marker+'                    path_source_stamp_ns=source_path_stamp_ns,\n')
    assert ast.dump(ast.parse(textwrap.dedent(body)))==ast.dump(ast.parse(textwrap.dedent(method(new,name)[2]))),name
    old=old[:start]+body+old[end:]
payload={members[0]:old}
# v19 added five dispatch tests; use that frozen file, not the local 5m-arrival tests.
old=(base/'test_graph_path_bridge_routing.py').read_text(encoding='utf-8')
assert 'test_synchronous_result_reuses_same_input_full_path_validation' in old
new=(repo/members[1]).read_text(encoding='utf-8')
first=new.index('    def test_status_records_exact_consumed_input_stamps_without_motion_changes(')
last=new.index('    def test_synchronous_result_reuses_same_input_full_path_validation(',first)
marker='    def test_synchronous_result_reuses_same_input_full_path_validation('
old=old.replace(marker,new[first:last]+marker,1)
needle='pose=SimpleNamespace(pose=pose(0., 0.).pose), header=Mock())'
assert old.count(needle)==1
old=old.replace(needle,'pose=SimpleNamespace(pose=pose(0., 0.).pose), header=pose(0., 0.).header)')
payload[members[1]]=old
hashes={name:hashlib.sha256((base/Path(name).name).read_bytes()).hexdigest() for name in members}
with zipfile.ZipFile(art/'v21_delta.zip','w',zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('base_hashes.json',json.dumps(hashes))
    for name,content in payload.items():
        ast.parse(content);archive.writestr(name,content)
print(json.dumps({name:hashlib.sha256(content.encode()).hexdigest() for name,content in payload.items()},indent=2))
