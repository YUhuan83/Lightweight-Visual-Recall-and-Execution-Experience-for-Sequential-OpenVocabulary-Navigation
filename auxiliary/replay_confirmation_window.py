import json,sqlite3
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import rclpy
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from beamng_wildos_adapter.goal_standoff import GoalStandoff
root=Path('/home/ly/execution_continuity_review_20261022/v29_range_consistency_r2')
trial=Path(json.loads((root/'field_result.json').read_text())['trial'])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid=db.execute("select id from topics where name='/spot1/object_localization'").fetchone()[0]
messages=[(t,json.loads(deserialize_message(blob,String).data)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
db.close();reports=[]
rclpy.init()
for window in (30.,60.):
    node=GoalStandoff();node.confirmation_window_sec=window;node.active_query=messages[0][1]['query'];node.query_started_ns=0
    node.confirmation_pub=Mock();node.publish_standoff=Mock(return_value='isolated_no_map')
    for received,d in messages:
        node.get_clock=lambda:S(now=lambda:Time(nanoseconds=received))
        odom=Odometry();odom.header.frame_id='spot1/odom'
        odom.header.stamp=Time(nanoseconds=d['observation_stamp_ns']).to_msg()
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=d['observer_position']
        q=odom.pose.pose.orientation;q.x,q.y,q.z,q.w=d['observer_orientation']
        node.on_odom(odom);node.on_localization(String(data=json.dumps(d)))
    outputs=[json.loads(c.args[0].data) for c in node.confirmation_pub.publish.call_args_list]
    reports.append(dict(window_sec=window,observations_retained=len(node.observations),confirmations=outputs,
                        scope='Actual ROS class, recorded localization messages, controlled clock/odom; no map/actuation/arrival'))
    node.destroy_node()
rclpy.shutdown()
assert not reports[0]['confirmations']
assert reports[1]['confirmations'][-1]['search_confirmed'] and reports[1]['confirmations'][-1]['inliers']==4
(root/'confirmation_window_replay.json').write_text(json.dumps(reports,indent=2))
(root/'localization_messages.json').write_text(json.dumps(messages,indent=2))
print(json.dumps(reports,indent=2))
