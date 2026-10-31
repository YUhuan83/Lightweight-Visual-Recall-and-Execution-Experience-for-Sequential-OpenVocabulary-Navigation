"""Bounded response waits, with uncertain motion held until terminal evidence."""
import math
import time
import uuid
import json
from std_msgs.msg import String
from .motion_path import maneuver_reached


class NavigationTransactions:
    def init_navigation_transactions(self):
        for name, default in (('follow_response_timeout_sec',5.),
                              ('follow_feedback_timeout_sec',10.),
                              ('graph_reset_timeout_sec',5.)):
            value = float(self.declare_parameter(name, default).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')
        self.follow_request = self.follow_active = None
        self.follow_quarantine = {}
        self.follow_stop_since = None
        self.follow_hold = False
        self.hold_owner = str(uuid.uuid4())
        self.hold_revision = 0
        self.hold_pub = self.create_publisher(String, '/wildos/execution_hold', 10)
        self.graph_reset_serial = 0
        self.graph_reset_current_serial = None
        self.graph_reset_request = None
        self.graph_reset_waiting = {}
        self.graph_reset_success = None
        self.graph_reset_attempts = 0
        self.graph_reset_retry_at = 0.
        self.graph_reset_exhausted = False
        self.graph_reset_intent_scope = None

    def publish_execution_hold(self, held):
        if held != self.follow_hold:
            self.follow_hold = held
            self.hold_revision += 1
        self.hold_pub.publish(String(data=json.dumps(dict(owner=self.hold_owner,
            revision=self.hold_revision, held=held, wall_time=time.time()))))

    def retire_follow_transactions(self):
        """Epoch invalidation does not prove a server-side action has stopped."""
        for transaction in (self.follow_request,self.follow_active):
            if transaction is not None:
                serial = transaction['serial']
                self.follow_quarantine.setdefault(serial, dict(handle=transaction.get('handle'),
                    terminal=False))
        if self.deferred_follow_result is not None and self.follow_active is not None:
            self.quarantine_follow_result(self.deferred_follow_result[0],self.follow_active['serial'])
        self.follow_request = self.follow_active = None
        self.active_follow_serial = None
        if self.follow_quarantine:
            self.follow_stop_since = None
            self.publish_execution_hold(True)

    def follow_transaction_timeout(self, reason):
        # No route rejection or escape: loss of protocol ownership is not
        # terrain evidence. The transport failure path retires/cancels motion.
        self.execution_transport_unavailable(reason)

    def on_follow_feedback(self, message, epoch, serial):
        active = self.follow_active
        if epoch == self.epoch and active is not None and active['serial'] == serial:
            active['feedback_at'] = time.monotonic()

    def quarantine_follow_acceptance(self, future, serial):
        entry = self.follow_quarantine.get(serial)
        if entry is None:
            return False
        try:
            handle = future.result()
        except Exception:
            return True  # Exception is not proof the server rejected the goal.
        if handle is None:
            return True
        if not handle.accepted:
            entry['terminal'] = True
        elif not entry.get('result_watching'):
            entry['handle'] = handle
            entry['result_watching'] = True
            try:
                handle.get_result_async().add_done_callback(
                    lambda f: self.quarantine_follow_result(f, serial))
            except Exception as exc:
                self.get_logger().warning('Cannot observe retired goal result: '+str(exc)[:120])
            self.cancel_goal(handle, 'retired_follow_path')
        return True

    def quarantine_follow_result(self, future, serial):
        entry = self.follow_quarantine.get(serial)
        if entry is None:
            return False
        try:
            terminal = future.result().status in (4,5,6)
        except Exception:
            terminal = False
        entry['terminal'] = terminal or entry['terminal']
        return True

    def monitor_follow_transactions(self):
        now = time.monotonic()
        if self.follow_quarantine:
            self.publish_execution_hold(True)
            # Do not turn a lost ACK into permission to retry. All uncertain
            # goals must terminate, including the superseded action.
            stopped = (all(v['terminal'] for v in self.follow_quarantine.values())
                       and self.input_freshness_issue() is None
                       and self.execution_transport_issue(for_stop=True) is None)
            if stopped:
                if self.carrier_model == 'ackermann':
                    from .execution_feedback import native_command_braking
                    stopped = (self.native_status is not None
                               and native_command_braking(self.native_status)
                               and abs(self.native_status['speed']) < .05)
                else:
                    velocity = self.odom.twist.twist
                    stopped = (math.hypot(velocity.linear.x,velocity.linear.y) < .05
                               and abs(velocity.angular.z) < .05)
            if not stopped:
                self.follow_stop_since = None
                return True
            if self.follow_stop_since is None:
                self.follow_stop_since = now
            if now-self.follow_stop_since < .5:
                return True
            self.follow_quarantine.clear()
            self.follow_stop_since = None
            self.publish_execution_hold(False)
            self.last_plan_start = self.last_plan_endpoint = None
            self.status('execution_recovered', reason='retired_actions_stopped')
            self.start_segment()
            return True
        request = self.follow_request
        if request is not None and now-request['started_at'] >= float(
                self.get_parameter('follow_response_timeout_sec').value):
            self.follow_transaction_timeout('follow_replacement_timeout' if request['replacement']
                                            else 'follow_acceptance_timeout')
            return True
        active = self.follow_active
        if active is not None:
            if now-active['feedback_at'] >= float(self.get_parameter('follow_feedback_timeout_sec').value):
                self.follow_transaction_timeout('follow_result_timeout')
                return True
            from .graph_path_bridge import xy,yaw
            endpoint = maneuver_reached(xy(self.odom.pose.pose), yaw(self.odom.pose.pose),
                active['target'],active['heading'],position_tolerance=active['xy_tolerance'],
                heading_tolerance=active['yaw_tolerance'])
            velocity = self.odom.twist.twist
            stopped = (math.hypot(velocity.linear.x,velocity.linear.y) < .05
                       and abs(velocity.angular.z) < .05)
            if endpoint and stopped:
                if active['endpoint_since'] is None:
                    active['endpoint_since'] = now
                if now-active['endpoint_since'] >= float(self.get_parameter('follow_response_timeout_sec').value):
                    self.follow_transaction_timeout('follow_result_timeout')
                    return True
            else:
                active['endpoint_since'] = None
        return False

    def request_graph_reset(self):
        self.query_reset_pending = True
        self.query_reset_inflight = False
        self.graph_reset_request = None
        self.graph_reset_current_serial = None
        self.graph_reset_success = None
        self.graph_reset_attempts = 0
        self.graph_reset_retry_at = 0.
        self.graph_reset_exhausted = False
        self.graph_reset_intent_scope = self.graph_reset_scope()

    def graph_reset_scope(self):
        return (self.query_generation,self.confirmation_not_before_ns)

    def graph_reset_may_dispatch(self):
        if not self.query_reset_pending:
            return False
        if self.graph_reset_intent_scope != self.graph_reset_scope():
            self.request_graph_reset()
        now = time.monotonic()
        if self.graph_reset_success == self.graph_reset_scope():
            if self.last_status.get('status') != 'graph_reset_failed':
                self.status('graph_reset_failed',reason='unresolved_prior_clear',query=self.query,
                            unresolved_requests=len(self.graph_reset_waiting))
            return False
        request = self.graph_reset_request
        if request is not None:
            if now-request['started_at'] < float(self.get_parameter('graph_reset_timeout_sec').value):
                return False
            self.graph_reset_request = None
            self.query_reset_inflight = False
            self.graph_reset_retry_at = now+1.
            self.status('graph_reset_timeout',attempt=self.graph_reset_attempts,query=self.query)
        if self.graph_reset_attempts >= 3:
            self.graph_reset_exhausted = True
            self.status('graph_reset_failed', reason='response_budget_exhausted',query=self.query,
                        unresolved_requests=len(self.graph_reset_waiting))
            return False
        return not self.query_reset_inflight and now >= self.graph_reset_retry_at

    def finish_graph_reset_barrier(self):
        # Trigger has no request identity on the server. An older unanswered
        # request could still clear a newly relayed destination. Drain *all*
        # issued responses before opening the route fence.
        if (self.graph_reset_waiting or self.graph_reset_success != self.graph_reset_scope()
                or self.instance_release_pending is not None):
            return False
        self.graph_reset_success = None
        self.graph_reset_request = None
        self.query_reset_pending = self.query_reset_inflight = False
        self.graph_reset_exhausted = False
        return True
