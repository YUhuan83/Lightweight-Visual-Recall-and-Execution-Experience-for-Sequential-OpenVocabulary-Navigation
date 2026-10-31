"""Profile recorded path admission; all action calls are isolated test doubles."""
import copy
import json
import math
import sqlite3
import time
import sys
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge, xy, yaw
from wildos_nav2_adapter.input_quality import stamp_ns

trial=Path('/home/ly/execution_continuity_review_20261022/v18_comparison_bound/full/trial_20261023_193041')
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
audit=json.loads((trial/'stale_odom_admission_1.json').read_text())
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def read(topic,kind):
    tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
    return [(t,deserialize_message(b,kind)) for t,b in db.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
paths=read('/wildos/full_planned_path',RosPath)
routes=read('/spot1/graphnav_planner/path',RosPath)
odoms=read('/spot1/odom',Odometry)
maps=read('/global_costmap/costmap',OccupancyGrid)
locals_=read('/local_costmap/costmap',OccupancyGrid)
metadata={d['stamp_ns']:d for d in [json.loads(m.data)
          for t,m in read('/spot1/graphnav_planner/route_status',String)]}
db.close()
rclpy.init()
output=[]
try:
    for evidence in audit['events']:
        planned=evidence['previous_planned'];event=evidence['event']
        # The callback publishes this full path only after its first validation.
        path_t,path=max(((t,m) for t,m in paths if t<planned['stamp_ns']),key=lambda p:p[0])
        odom=next(m for t,m in odoms if stamp_ns(m)==planned['actual_start_stamp_ns'])
        route=max(((t,m) for t,m in routes if t<path_t and m.poses and math.dist(
            xy(m.poses[-1].pose),planned['graph_goal'])<.01),key=lambda p:p[0])[1]
        start_t=stamp_ns(odom)
        grid=max(((t,m) for t,m in maps if t<=start_t),key=lambda p:p[0])[1]
        local=max(((t,m) for t,m in locals_ if t<=start_t),key=lambda p:p[0])[1]
        runs=[]
        for repeat in range(2):
            node=GraphPathBridge()
            calls=[]
            try:
                node.carrier_model='ackermann';node.minimum_turning_radius=8.;node.planner_id='CarLike'
                node.follow_enabled=True;node.odom=copy.deepcopy(odom)
                node.grid=grid;node.local_grid=local
                node.route=node.latest_route=copy.deepcopy(route)
                node.route_endpoint=xy(route.poses[-1].pose)
                node.route_metadata=metadata;node.route_complete=metadata.get(stamp_ns(route),{}).get('complete',False)
                now=time.monotonic()
                node.odom_received_at=now-.08
                node.grid_received_at=now-max(0.,(start_t-stamp_ns(grid))/1e9)
                node.local_grid_received_at=now-max(0.,(start_t-stamp_ns(local))/1e9)
                node.execution_transport_issue=Mock(return_value=None)
                node.plan_client=Mock();node.follow_client=Mock()
                node.follow_client.send_goal_async.return_value=Future()
                node.status_pub=Mock();node.full_path_pub=Mock();node.path_pub=Mock()
                target=xy(path.poses[-1].pose)
                node.candidates=[(*target,yaw(path.poses[-1].pose),math.dist(xy(odom.pose.pose),target))]
                node.candidate_index=0;node.active_plan_serial=1;node.pending=True
                node.follow_quarantine={7:dict(handle=Mock(),terminal=False)}
                for name in ('validate_path','initial_tracking_issue','validate_local_path','tracking_issue'):
                    original=getattr(node,name)
                    def timed(*args,_name=name,_fn=original,**kwargs):
                        before=time.perf_counter()
                        result=_fn(*args,**kwargs)
                        calls.append(dict(method=_name,poses=len(args[0]),seconds=time.perf_counter()-before,result=result))
                        return result
                    setattr(node,name,timed)
                future=Future();future.set_result(S(status=4,result=S(path=copy.deepcopy(path))))
                before=time.perf_counter()
                node.on_plan_result(future,node.epoch,0,1)
                elapsed=time.perf_counter()-before
                actions_before_stop=node.follow_client.send_goal_async.call_count
                prepared=getattr(node,'prepared_stop_plan',None) is not None
                node.follow_quarantine.clear()
                now=time.monotonic()
                node.odom_received_at=now-.08
                node.grid_received_at=now-.1;node.local_grid_received_at=now-.1;node.grid_arrived_at=now
                before=time.perf_counter();node.start_segment();resume_s=time.perf_counter()-before
                runs.append(dict(repeat=repeat,elapsed_s=elapsed,calls=calls,
                    prepared=prepared,actions_before_stop=actions_before_stop,resume_s=resume_s,
                    new_planning_requests=node.plan_client.send_goal_async.call_count,
                    status=node.last_status,action_dispatches=node.follow_client.send_goal_async.call_count,
                    freshness_after=node.input_freshness_issue()))
            finally:
                node.destroy_node()
        output.append(dict(event_elapsed=event['elapsed'],path_poses=len(path.poses),
            path_bag_time_ns=path_t,source_odom_ns=start_t,source_map_ns=stamp_ns(grid),runs=runs))
    (Path(sys.argv[1])/'stop_plan_recorded_replay.json').write_text(json.dumps(output,indent=2))
    print(json.dumps(output,indent=2))
finally:
    rclpy.shutdown()
