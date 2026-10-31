"""Apply only serialized odometry ingress changes to frozen v21 r2."""
import ast
import hashlib
import json
import zipfile
from pathlib import Path

repo=Path(r'C:\Users\Yu\Desktop\拟真');art=repo/'test_artifacts/fresh_command_retry_20261023'
base=art/'v21_base'
members=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
         'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
         'nav2_adapter/wildos_nav2_adapter/target_observation.py',
         'nav2_adapter/test/test_graph_path_bridge_routing.py',
         'nav2_adapter/test/test_native_status_executor_ros.py']
payload={}
old=(base/'graph_path_bridge.py').read_text(encoding='utf-8')
new=(repo/members[0]).read_text(encoding='utf-8')
old=old.replace('from copy import deepcopy','from collections import deque\nfrom copy import deepcopy\nfrom threading import Lock',1)
old=old.replace('from rclpy.action import ActionClient','from rclpy.action import ActionClient\nfrom rclpy.callback_groups import MutuallyExclusiveCallbackGroup',1)
start=new.index('        self.odom_input_lock = Lock()')
end=new.index('        self.create_subscription(OccupancyGrid',start)
needle='        self.create_subscription(Odometry, "/spot1/odom", self.on_odom, qos_profile_sensor_data)\n'
assert old.count(needle)==1;old=old.replace(needle,new[start:end],1)
start=new.index('    def receive_odometry(');end=new.index('    def on_odom(',start)
old=old.replace('    def on_odom(',new[start:end]+'    def on_odom(',1)
for function in ('retry_when_planner_starts','monitor_inputs','reset','on_object_confirmation','on_route',
                 'start_segment','execute_checked_path','send_motion_segment','on_follow_accepted','on_follow_result'):
    start=old.index('    def '+function+'(');end=old.index('\n',start)+1
    old=old[:end]+'        self.consume_pending_odometry()\n'+old[end:]
needle='        """Synchronous handoff from whole-path checks to segment admission."""\n'
assert old.count(needle)==1;old=old.replace(needle,needle+'        self.consume_pending_odometry()\n')
start=old.index('    def on_plan_result(');end=old.index('    def exploration_motion_issue(',start)
body=old[start:end];needle='        self.active_plan_serial = None\n'
assert body.count(needle)==1;body=body.replace(needle,'        self.consume_pending_odometry()\n'+needle)
payload[members[0]]=old[:start]+body+old[end:]
for name,function in ((members[1],'activate_visible_approach'),(members[2],'monitor_observation')):
    old=(base/Path(name).name).read_text(encoding='utf-8')
    marker='    def '+function+'(self):\n';assert old.count(marker)==1
    payload[name]=old.replace(marker,marker+'        self.consume_pending_odometry()\n')

old=(base/'test_graph_path_bridge_routing.py').read_text(encoding='utf-8')
new=(repo/members[3]).read_text(encoding='utf-8')
start=new.index('    def odometry_message(');end=new.index('    def test_planning_evidence_matches_the_actual_action_goal_heading(',start)
marker='    def test_planning_evidence_matches_the_actual_action_goal_heading('
assert old.count(marker)==1;payload[members[3]]=old.replace(marker,new[start:end]+marker)

old=(base/'test_native_status_executor_ros.py').read_text(encoding='utf-8')
new=(repo/members[4]).read_text(encoding='utf-8')
old=old.replace('from std_msgs.msg import String','from std_msgs.msg import String\nfrom nav_msgs.msg import Odometry\nfrom rclpy.qos import qos_profile_sensor_data',1)
old=old.replace('from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge',
                'from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge\nfrom wildos_nav2_adapter.input_quality import stamp_ns',1)
start=new.index('    def odometry_during_navigation_work(');end=new.index('    def test_navigation_result_waits_for_state_group_and_checks_retired_epoch(',start)
marker='    def test_navigation_result_waits_for_state_group_and_checks_retired_epoch('
assert old.count(marker)==1;payload[members[4]]=old.replace(marker,new[start:end]+marker)
hashes={name:hashlib.sha256((base/Path(name).name).read_bytes()).hexdigest() for name in members}
with zipfile.ZipFile(art/'v22_delta.zip','w',zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('base_hashes.json',json.dumps(hashes))
    for name,content in payload.items():
        ast.parse(content);archive.writestr(name,content)
print(json.dumps({name:hashlib.sha256(content.encode()).hexdigest() for name,content in payload.items()},indent=2))
