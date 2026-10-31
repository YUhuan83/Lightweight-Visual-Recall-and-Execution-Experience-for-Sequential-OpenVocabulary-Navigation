"""Replay actual selected paths against the earlier recorded comparison maps."""
import copy
import json
import sqlite3
import time
from pathlib import Path
from unittest.mock import Mock

import rclpy
from graphnav_msgs.msg import ExplorationChoice
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge, xy, yaw
from wildos_nav2_adapter.graph_route import maneuver_cost
from wildos_nav2_adapter.input_quality import stamp_ns
from wildos_nav2_adapter.motion_path import first_section_retreat_cost


def replay(trial_audit):
    trial=Path(trial_audit['trial'])
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    def read(topic,kind):
        identifier=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
        return [(t,deserialize_message(b,kind)) for t,b in db.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',(identifier,))]
    plans=[json.loads(m.data) for t,m in read('/wildos/plan_status',String)]
    choices=read('/spot1/graphnav_planner/vehicle_choice',ExplorationChoice)
    paths=read('/wildos/full_planned_path',RosPath)
    graph_paths=read('/spot1/graphnav_planner/path',RosPath)
    metadata={d['stamp_ns']:d for d in [json.loads(m.data)
              for t,m in read('/spot1/graphnav_planner/route_status',String)]}
    maps=read('/global_costmap/costmap',OccupancyGrid)
    odom=read('/spot1/odom',Odometry)
    out=[]
    for group in trial_audit['prunable']:
        identity=group['identity']
        event=next(p for p in plans if p.get('exploration_id')==identity
            and p['status']=='exploration_cost_evaluated' and p['candidate_uuid']==group['after_uuid'])
        choice_t,choice=next((t,m) for t,m in choices if m.exploration_id==identity)
        options=group['options']
        option=next(o for o in options if o['uuid']==choice.chosen_uuid)
        candidates=[]
        for t,path in paths:
            if not choice_t<=t<=choice_t+2_000_000_000 or not path.poses:
                continue
            points=[xy(p.pose) for p in path.poses]
            cost=maneuver_cost(points)+.25*first_section_retreat_cost(points,option['goal'][:2])
            if abs(cost-choice.vehicle_cost)<1e-6:
                candidates.append((t,path,cost))
        if not candidates:
            out.append(dict(identity=identity,skipped='selected path not recorded within 2 seconds'))
            continue
        t,path,cost=candidates[0]
        grid=next(m for t,m in maps if stamp_ns(m)==event['costmap_stamp_ns'])
        odom_msg=max(((t,m) for t,m in odom if t<=event['stamp_ns']),key=lambda p:p[0])[1]
        node=GraphPathBridge()
        try:
            node.carrier_model='ackermann';node.minimum_turning_radius=8.;node.follow_enabled=True
            node.planner_id='CarLike'
            node.grid=grid;node.odom=odom_msg
            node.route=copy.deepcopy(next(m for t,m in graph_paths if stamp_ns(m)==stamp_ns(choice)))
            node.route_endpoint=xy(node.route.poses[-1].pose)
            node.route_metadata=metadata
            node.status_pub=Mock();node.frontier_choice_pub=Mock();node.try_candidate=Mock()
            state=dict(identity=identity,header=choice.header,endpoint=choice.route_endpoint,
                origin=choice.evaluation_start,origin_heading=yaw(choice.evaluation_start),
                epoch=node.epoch,started=time.monotonic(),options=options,
                original_candidates=[(*option['goal'],0.)],
                results=[(option['benefit']-.004*cost,option,cost,path)])
            node.frontier_comparison=state
            node.candidates=[(*o['goal'],0.) for o in options]
            node.candidate_index=next(i for i,o in enumerate(options) if o['uuid']==group['after_uuid'])
            reason=node.frontier_path_issue(path,option,state)
            current=node.frontier_comparison_current()
            started=time.monotonic()
            node.advance_frontier_comparison()
            dt=time.monotonic()-started
            selected=(node.frontier_choice_pub.publish.call_args.args[0].chosen_uuid
                if node.frontier_choice_pub.publish.called else None)
            out.append(dict(identity=identity,actual_path_poses=len(path.poses),
                recorded_cost=cost,recorded_choice=choice.chosen_uuid,replay_choice=selected,
                map_stamp_ns=event['costmap_stamp_ns'],path_issue=reason,current_context=current,
                remaining_planner_calls=node.try_candidate.call_count,
                callback_time_s=dt,passed=(selected==choice.chosen_uuid and current and reason is None)))
        finally:
            node.destroy_node()
    db.close()
    return dict(trial=str(trial),results=out)


rclpy.init()
try:
    result=[replay(t) for t in json.loads(Path('/tmp/v17_comparison_bounds_1.json').read_text())]
    destination=Path('/home/ly/execution_continuity_review_20261022/v18_comparison_bound/recorded_comparison_replay.json')
    destination.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    assert all(row.get('passed') for trial in result for row in trial['results'])
finally:
    rclpy.shutdown()
