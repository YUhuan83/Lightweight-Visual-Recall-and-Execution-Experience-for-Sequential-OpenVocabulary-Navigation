"""Bounded, read-only frontier comparison owned by the normal planner transaction."""
from collections import deque
from copy import deepcopy
import json
import math
import time
from graphnav_msgs.msg import ExplorationChoice
from std_msgs.msg import String
from .graph_route import distance, maneuver_cost
from .input_quality import stamp_ns

COMPARISON_START_TOLERANCE_M = .05
COMPARISON_GOAL_TOLERANCE_M = 1.


class ExplorationCostComparison:
    def init_exploration_cost(self):
        self.frontier_comparison = None
        self.frontier_choice = None
        self.frontier_compared = deque(maxlen=64)
        self.frontier_protocol_failures = 0
        self.frontier_choice_pub = self.create_publisher(
            ExplorationChoice, '/spot1/graphnav_planner/vehicle_choice', 10)
        self.create_subscription(String, '/spot1/graphnav_planner/selection_ack',
                                 self.on_frontier_choice_ack, 10)

    def clear_exploration_comparison(self):
        self.frontier_comparison = self.frontier_choice = None

    def begin_exploration_comparison(self):
        from .graph_path_bridge import xy, yaw
        # start_segment has built a new normal candidate set. A stale result
        # retained after a transport/epoch failure cannot restore an older set.
        if self.frontier_comparison is not None:
            self.frontier_compared.append(self.frontier_comparison['identity'])
            self.frontier_comparison=None
        metadata = self.exploration_metadata(self.route)
        if (self.carrier_model != 'ackermann' or not self.follow_enabled or self.extending_path
                or self.following or self.follow_request is not None or self.follow_active is not None
                or self.follow_quarantine or self.recovering or self.memory_selecting
                or self.frontier_protocol_failures >= 2 or not metadata
                or metadata.get('complete') is not True or metadata.get('vehicle_cost_selected') is True
                or metadata['exploration_id'] in self.frontier_compared):
            return False
        options = metadata.get('vehicle_options')
        if not isinstance(options,list) or not 2 <= len(options) <= 4:
            return False
        speed=math.hypot(self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y)
        if not math.isfinite(speed) or speed > .15 or self.local_freshness_issue() is not None:
            return False
        start=xy(self.odom.pose.pose)
        self.sync_failed_goal_scope(metadata['exploration_id'])
        parsed=[]
        try:
            for option in options:
                uid,goal,benefit=option['uuid'],option['goal'],option['benefit']
                if (not isinstance(uid,str) or not uid or len(uid)>128 or len(goal)!=3
                        or not all(math.isfinite(v) for v in (*goal,benefit))
                        or distance(start,goal[:2]) > self.max_segment):
                    return False
                parsed.append(dict(uuid=uid,goal=tuple(goal),benefit=float(benefit)))
        except (KeyError,TypeError,ValueError,OverflowError):
            return False
        if (len({o['uuid'] for o in parsed}) != len(parsed)
                or not any(distance(o['goal'],metadata['goal']) < 1e-5 for o in parsed)):
            return False
        # GraphNav can offer a node that is topologically reachable while the
        # actual vehicle body cannot occupy its endpoint pose. The global
        # costmap then rejects every ComputePath attempt for that option, and
        # the bridge spends a full planner search per infeasible frontier.
        # Keep the GraphNav identity/benefit contract over the original list,
        # but only plan options whose real footprint fits on this map version.
        if (self.grid is None or self.grid.header.frame_id != self.route.header.frame_id):
            return False
        feasible=[o for o in parsed
                  if not self.goal_recently_failed(start,o['goal'][:2],o['goal'][2])
                  and self.vehicle_pose_feasible(o['goal'][:2],o['goal'][2])]
        skipped=[o['uuid'] for o in parsed if o['uuid'] not in {f['uuid'] for f in feasible}]
        if skipped:
            self.get_logger().info(
                "Exploration options skipped (cached failure or no clear goal pose): " +
                ",".join(skipped))
        if len(feasible) < 2:
            # A single option is not a comparison. Preserve the normal route
            # candidate path and its existing planner/geometry checks.
            return False
        self.frontier_comparison=dict(identity=metadata['exploration_id'],
            header=deepcopy(self.route.header),endpoint=deepcopy(self.route.poses[-1].pose.position),
            origin=deepcopy(self.odom.pose.pose),origin_heading=yaw(self.odom.pose.pose),
            epoch=self.epoch,started=time.monotonic(),options=feasible,results=[],
            skipped_options=skipped,original_candidates=self.candidates)
        self.candidates=[(*o['goal'],distance(start,o['goal'])) for o in feasible]
        return True

    def frontier_comparison_current(self):
        from .graph_path_bridge import xy, yaw
        state=self.frontier_comparison
        return (state is not None and state['epoch']==self.epoch
            and self.odom is not None and self.grid is not None
            and self.odom.header.frame_id == self.grid.header.frame_id == state['header'].frame_id
            and distance(xy(self.odom.pose.pose),xy(state['origin'])) <= .25
            and abs(math.remainder(yaw(self.odom.pose.pose)-state['origin_heading'],2*math.pi)) <= .1
            and time.monotonic()-state['started'] <= 12.)

    def record_frontier_cost(self, path):
        from .graph_path_bridge import xy, yaw
        from .motion_path import first_section_retreat_cost
        state=self.frontier_comparison
        if not self.frontier_comparison_current():
            self.finish_frontier_comparison(aborted=True)
            return
        index=self.candidate_index
        option=state['options'][index]
        points=[xy(p.pose) for p in path.poses]
        reason=self.frontier_path_issue(path,option,state)
        if reason is None:
            cost=maneuver_cost(points)
            cost += .25 * first_section_retreat_cost(points, option['goal'][:2])
            length=sum(distance(a,b) for a,b in zip(points,points[1:]))
            state['results'].append((option['benefit']-.004*cost,option,cost,path))
            self.status('exploration_cost_evaluated',candidate_uuid=option['uuid'],
                        candidate_goal=list(option['goal']),path_length_m=length,
                        vehicle_cost_m=cost,comparison_only=True,
                        costmap_stamp_ns=stamp_ns(self.grid))
        self.advance_frontier_comparison(reason)

    def frontier_path_issue(self,path,option,state):
        from .graph_path_bridge import xy,yaw
        points=[xy(p.pose) for p in path.poses]
        headings=[yaw(p.pose) for p in path.poses]
        target=option['goal'][:2]
        reason=self.validate_path(points,headings,[xy(state['origin']),target],self.grid,target,
            start=xy(state['origin']),target_heading=option['goal'][2])
        if reason is None and (distance(points[0],xy(state['origin']))>COMPARISON_START_TOLERANCE_M
                or abs(math.remainder(headings[0]-state['origin_heading'],2*math.pi))>.1
                or distance(points[-1],target)>COMPARISON_GOAL_TOLERANCE_M
                or abs(math.remainder(headings[-1]-option['goal'][2],2*math.pi))>.35):
            reason='comparison_pose_mismatch'
        reason=reason or self.exploration_motion_issue(points)
        reason=reason or self.initial_tracking_issue(points,headings)
        return reason

    def advance_frontier_comparison(self, reason=None):
        if reason:
            self.status('exploration_cost_rejected',candidate_index=self.candidate_index,
                        reason=reason,comparison_only=True)
        self.pending=False
        self.plan_handle=None
        self.plan_request_started_at=None
        self.active_plan_serial=None
        self.candidate_index+=1
        if (not self.frontier_comparison_current() or self.candidate_index>=len(self.candidates)):
            self.finish_frontier_comparison(aborted=not self.frontier_comparison_current())
        elif self.remaining_frontier_costs_dominated():
            self.finish_frontier_comparison()
        else:
            self.try_candidate()

    def remaining_frontier_costs_dominated(self):
        """Stop only when no remaining option can beat a still-valid path.

        Vehicle cost is path length plus nonnegative maneuver/retreat costs.
        Its lower bound must allow the same 5 cm start and 1 m end tolerance
        as frontier_path_issue. Keep strict ties and the original option order.
        """
        from .graph_path_bridge import xy
        state=self.frontier_comparison
        remaining=state['options'][self.candidate_index:]
        upper=[option['benefit']-.004*max(0.,
               distance(xy(state['origin']),option['goal'][:2])
               -COMPARISON_GOAL_TOLERANCE_M-COMPARISON_START_TOLERANCE_M)
               for option in remaining]
        if not upper:
            return False
        # Only repeat the geometric checks when the cheap bound can actually
        # save a planner call; otherwise preserve normal comparison latency.
        eligible=sorted((result for result in state['results']
                         if max(upper) < result[0]-1e-9),
                        key=lambda result:result[0],reverse=True)
        best=next((result for result in eligible
                   if self.frontier_path_issue(result[3],result[1],state) is None),None)
        if best is None:
            return False
        self.status('exploration_cost_bound_complete',comparison_only=True,
                    candidate_uuid=best[1]['uuid'],score=best[0],
                    skipped_candidates=[option['uuid'] for option in remaining],
                    remaining_score_upper_bounds=upper,
                    costmap_stamp_ns=stamp_ns(self.grid))
        return True

    def finish_frontier_comparison(self, aborted=False):
        state,self.frontier_comparison=self.frontier_comparison,None
        if state is None:
            return
        self.frontier_compared.append(state['identity'])
        self.pending=False
        self.candidates=state['original_candidates']
        self.candidate_index=0
        self.failure_reasons=[]
        self.normal_best=None
        if not aborted:
            # Every retained path must still be valid on this common latest
            # map. Earlier feasibility cannot survive a newly observed obstacle.
            state['results']=[r for r in state['results']
                if self.frontier_path_issue(r[3],r[1],state) is None]
        if aborted or not state['results']:
            # A failed cost comparison is neither an obstacle label nor an
            # exhausted normal route. Preserve the normal heading/prefix checks.
            self.status('exploration_cost_unavailable',aborted=aborted,comparison_only=True)
            self.try_candidate()
            return
        _,option,cost,path=max(state['results'],key=lambda r:r[0])
        choice=ExplorationChoice()
        choice.header=state['header']
        choice.exploration_id=state['identity']
        choice.route_endpoint=state['endpoint']
        choice.evaluation_start=state['origin']
        choice.chosen_uuid=option['uuid']
        choice.vehicle_cost=cost
        now=time.monotonic()
        self.frontier_choice=dict(message=choice,epoch=self.epoch,started=now,sent=now,
            path=deepcopy(path),option=deepcopy(option),origin=deepcopy(state['origin']),
            origin_heading=state['origin_heading'])
        self.frontier_choice_pub.publish(choice)
        self.status('frontier_selection_waiting',candidate_uuid=option['uuid'],vehicle_cost_m=cost,
                    costmap_stamp_ns=stamp_ns(self.grid),source_stamp_ns=stamp_ns(choice))

    def on_frontier_choice_ack(self, message):
        waiting=self.frontier_choice
        if waiting is None or waiting['epoch']!=self.epoch:
            return
        try:
            data=json.loads(message.data)
            choice=waiting['message']
            age=(self.get_clock().now().nanoseconds-int(data['stamp_ns']))*1e-9
            if (data['exploration_id']!=choice.exploration_id
                    or int(data['source_stamp_ns'])!=stamp_ns(choice)
                    or not isinstance(data['accepted'],bool) or not 0.<=age<=5.):
                return
        except (ValueError,TypeError,KeyError,OverflowError):
            return
        if not data['accepted']:
            self.frontier_choice=None
            self.frontier_protocol_failures+=1
            self.last_plan_start=self.last_plan_endpoint=None
            self.start_segment()
        # Acceptance is completed by a fresh graph route with selected=True.
        # An ACK alone never authorizes an alternative cached path to execute.

    def wait_for_frontier_choice(self, message, metadata):
        waiting=self.frontier_choice
        if waiting is None:
            return False
        choice=waiting['message']
        if (metadata is not None and metadata.get('vehicle_cost_selected') is True
                and metadata.get('exploration_id','').split(':')[0] == choice.chosen_uuid
                and stamp_ns(message)>stamp_ns(choice)):
            self.frontier_choice=None
            self.last_plan_start=self.last_plan_endpoint=None
            return False
        return True

    def execute_frontier_choice_path(self, waiting, metadata):
        """Reuse an evaluated path only after its selected graph route commits.

        The choice ACK alone is insufficient. on_route first admits the fresh
        graph owner and installs its route, then calls this method once. Changed
        geometry, pose, inputs or ownership use the normal planning pipeline.
        """
        from .graph_path_bridge import xy, yaw
        if (waiting is None or metadata is None or self.frontier_choice is not None
                or waiting['epoch'] != self.epoch
                or time.monotonic()-waiting['started'] >= 5.
                or self.carrier_model != 'ackermann' or not self.follow_enabled
                or self.following or self.pending or self.follow_request is not None
                or self.follow_active is not None or self.follow_quarantine or self.follow_hold
                or self.recovering or self.memory_selecting or self.query_reset_pending
                or self.expected is not None or self.memory_goal is not None
                or metadata.get('complete') is not True
                or metadata.get('vehicle_cost_selected') is not True
                or 'path' not in waiting or 'option' not in waiting):
            return False
        option = waiting['option']
        choice = waiting['message']
        heading = metadata.get('observation_heading')
        if (metadata.get('exploration_id', '').split(':')[0] != choice.chosen_uuid
                or distance(metadata['goal'], option['goal'][:2]) > 1e-5
                or not isinstance(heading, (int, float)) or not math.isfinite(heading)
                or abs(math.remainder(heading-option['goal'][2], 2*math.pi)) > 1e-6
                or stamp_ns(self.route) <= stamp_ns(choice)
                or self.odom is None or self.grid is None or self.local_grid is None
                or self.odom.header.frame_id != self.grid.header.frame_id
                or self.grid.header.frame_id != self.local_grid.header.frame_id
                or self.route.header.frame_id != waiting['path'].header.frame_id
                or self.input_freshness_issue() is not None
                or self.local_freshness_issue() is not None
                or self.execution_transport_issue() is not None):
            return False
        current = xy(self.odom.pose.pose)
        speed = math.hypot(self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y)
        if (not math.isfinite(speed) or speed > .15
                or distance(current, xy(waiting['origin'])) > .25
                or abs(math.remainder(yaw(self.odom.pose.pose)-waiting['origin_heading'],
                                      2*math.pi)) > .1
                or self.frontier_path_issue(waiting['path'], option, waiting) is not None):
            return False
        # execute_checked_path repeats current-route/map/tracking admission and
        # dispatches through the existing local sweep, cusp and action guards.
        self.last_plan_start = current
        self.last_plan_endpoint = tuple(metadata['goal'])
        # Any dispatch-time rejection belongs to the committed owner. Never
        # advance the original frontier's pre-comparison candidate list.
        self.candidates = [(*option['goal'], distance(current, option['goal'][:2]))]
        self.candidate_index = 0
        self.failure_reasons = []
        self.normal_best = None
        self.full_path_pub.publish(waiting['path'])
        self.execute_checked_path(waiting['path'], option['goal'][:2])
        return True

    def frontier_choice_for_retirement(self, data):
        """Keep a pending selection through retirement of its original owner.

        C++ retires that owner before publishing the selected replacement. This
        only preserves evidence; a fresh matching route must still commit it.
        """
        waiting = self.frontier_choice
        if (waiting is None or waiting['epoch'] != self.epoch
                or time.monotonic()-waiting['started'] >= 5.
                or data.get('reason') != 'vehicle_cost_selection'
                or data['exploration_id'] != waiting['message'].exploration_id
                or int(data['stamp_ns']) < stamp_ns(waiting['message'])
                or self.following or self.pending or self.follow_request is not None
                or self.follow_active is not None or self.follow_quarantine or self.follow_hold
                or self.recovering or self.memory_selecting or self.query_reset_pending
                or self.expected is not None or self.memory_goal is not None):
            return None
        return waiting

    def monitor_frontier_choice(self):
        waiting=self.frontier_choice
        if waiting is None:
            return False
        if waiting['epoch']!=self.epoch:
            self.frontier_choice=None
            return False
        now=time.monotonic()
        if now-waiting['started']>=5.:
            # A missing ACK cannot prove whether the graph committed. Drain a
            # real clear-goal barrier before planning again, using the existing
            # query/reset protocol; no speculative execution of either target.
            self.frontier_protocol_failures+=1
            self.reset(preserve_execution_corridor=True)
            self.request_graph_reset()
            self.retry_graph_reset()
            self.status('frontier_selection_timeout',requires_graph_reset=True)
        elif now-waiting['sent']>=1.:
            waiting['sent']=now
            self.frontier_choice_pub.publish(waiting['message'])
        return True
