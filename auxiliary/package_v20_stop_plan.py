"""Freeze only the stop/compute overlap changes on top of v19."""
import ast
import difflib
import hashlib
import json
import zipfile
from pathlib import Path

repo=Path(r'C:\Users\Yu\Desktop\拟真')
art=repo/'test_artifacts/fresh_command_retry_20261023'
base=art/'v19_base'
members=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
         'nav2_adapter/wildos_nav2_adapter/continuous_path.py',
         'nav2_adapter/test/test_motion_intent_handoff_ros.py',
         'nav2_adapter/test/test_committed_frontier_path_ros.py']
payload={}
def method(text,name):
    start=text.index('    def '+name+'(')
    end=text.find('\n    def ',start+5)
    assert end!=-1
    return start,end,text[start:end]
def replace_method(old,new,name):
    start,end,_=method(old,name)
    return old[:start]+method(new,name)[2]+old[end:]

old=(base/Path(members[0]).name).read_text(encoding='utf-8')
new=(repo/members[0]).read_text(encoding='utf-8')
for name in ('monitor_inputs','start_segment','dispatch_validated_path'):
    old=replace_method(old,new,name)
marker='    def reset(self, preserve_execution_corridor=False):\n'
assert old.count(marker)==1
payload[members[0]]=old.replace(marker,marker+'        self.prepared_stop_plan = None\n')

old=(base/Path(members[1]).name).read_text(encoding='utf-8')
new=(repo/members[1]).read_text(encoding='utf-8')
old=old.replace('        self.prepared_extension = None\n',
                '        self.prepared_extension = None\n        self.prepared_stop_plan = None\n',1)
start=new.index('    def stop_plan_scope(');end=new.index('    def maybe_extend_path(')
payload[members[1]]=old.replace('    def maybe_extend_path(',new[start:end]+'    def maybe_extend_path(',1)

old=(base/Path(members[2]).name).read_text(encoding='utf-8')
new=(repo/members[2]).read_text(encoding='utf-8')
start=new.index('class StopPlanPreparationTests(');end=new.index('class MotionIntentHandoffTests(')
payload[members[2]]=old.replace('from std_msgs.msg import String\n',
    'from std_msgs.msg import String\nimport rclpy\n').replace('class MotionIntentHandoffTests(',
    new[start:end]+'class MotionIntentHandoffTests(',1)

old=(base/Path(members[3]).name).read_text(encoding='utf-8')
new=(repo/members[3]).read_text(encoding='utf-8')
start,end,body=method(old,'test_replacement_route_waits_for_retired_action_stop_barrier')
replacement=method(new,'test_replacement_route_computes_while_execution_waits_for_stop_barrier')[2]
payload[members[3]]=old[:start]+replacement+old[end:]

hashes={name:hashlib.sha256((base/Path(name).name).read_bytes()).hexdigest() for name in members}
with zipfile.ZipFile(art/'v20_delta.zip','w',zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('base_hashes.json',json.dumps(hashes))
    for name,content in payload.items():
        ast.parse(content)
        archive.writestr(name,content)
        before=(base/Path(name).name).read_text(encoding='utf-8')
        diff=''.join(difflib.unified_diff(before.splitlines(True),content.splitlines(True),fromfile='v19/'+name,tofile='v20/'+name))
        (art/('v20_'+Path(name).stem+'.diff')).write_text(diff,encoding='utf-8')
print(json.dumps({name:hashlib.sha256(content.encode()).hexdigest() for name,content in payload.items()},indent=2))
