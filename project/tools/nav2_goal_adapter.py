#!/usr/bin/env python3
"""Standalone PoseStamped -> NavigateToPose bridge; requires a ROS 2 runtime.

No TF conversion, Nav2 installation, lifecycle management, or velocity bridge.
Transport ambiguity fails closed. Shutdown cancellation is best effort only.
"""
import math
import signal
import time

from goal_policy import Goal, GoalPolicy, validate


def main():
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    class Adapter(Node):
        def __init__(self):
            super().__init__('nav2_goal_adapter')
            defaults = {
                'goal_topic': '/rover/goal_pose',
                'action_name': '/navigate_to_pose',
                'expected_frame': 'odom',
                'max_goal_age_sec': 5.0,
                'min_distance_m': 0.1,
                'min_heading_rad': 0.1,
                'retry_period_sec': 0.5,
                'shutdown_timeout_sec': 3.0,
            }
            config = {}
            for name, default in defaults.items():
                config[name] = self.declare_parameter(name, default).value
            for name in ('goal_topic', 'action_name', 'expected_frame'):
                if not isinstance(config[name], str) or not config[name].strip():
                    raise ValueError(f'{name} must be nonempty')
            for name, default in defaults.items():
                if isinstance(default, float):
                    if not math.isfinite(config[name]) or config[name] <= 0:
                        raise ValueError(f'{name} must be finite and positive')
            self.config = config
            self.policy = GoalPolicy(config['min_distance_m'], config['min_heading_rad'])
            self.handle = None
            self.last_warning = {}
            self.client = ActionClient(self, NavigateToPose, config['action_name'])
            self.subscription = self.create_subscription(
                PoseStamped, config['goal_topic'], self.receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(config['retry_period_sec'], self.tick,
                                          clock=self.steady_clock)
            self.get_logger().info(
                f"Bridging {config['goal_topic']} -> {config['action_name']}; "
                f"only frame={config['expected_frame']}; no TF conversion/verification")

        def warn(self, key, text):
            now = time.monotonic()
            if now - self.last_warning.get(key, float('-inf')) >= 5.0:
                self.last_warning[key] = now
                self.get_logger().warning(text)

        def invalid(self, goal):
            return validate(goal, self.get_clock().now().nanoseconds,
                            self.config['expected_frame'], self.config['max_goal_age_sec'])

        def receive(self, msg):
            if self.policy.stopping:
                return
            stamp = msg.header.stamp
            if not 0 <= stamp.nanosec < 1_000_000_000:
                self.warn('invalid', 'Rejected goal: invalid nanosecond stamp')
                return
            p, q = msg.pose.position, msg.pose.orientation
            goal = Goal(msg.header.frame_id, stamp.sec * 1_000_000_000 + stamp.nanosec,
                        (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            reason = self.invalid(goal)
            if reason:
                self.warn('invalid', f'Rejected goal: {reason}')
                return
            if goal.stamp_ns < self.policy.latest_stamp:
                self.warn('order', 'Rejected out-of-order goal; clock reset requires bridge restart')
                return
            self.policy.offer(goal)
            # Dispatch only on the timer, bounding action traffic under fast input.

        def tick(self):
            policy = self.policy
            if policy.pending is not None:
                reason = self.invalid(policy.pending)
                if reason:
                    policy.pending = None
                    self.warn('expired', f'Dropped pending goal: {reason}')
            token = policy.begin_cancel()
            if token is not None:
                self.get_logger().info(f'Cancel requested for task {token}; awaiting terminal result')
                try:
                    self.handle.cancel_goal_async().add_done_callback(
                        lambda future, t=token: self.canceled(t, future))
                except Exception as exc:
                    self.failed(token, 'cancel request', exc)
                return
            if policy.stopping or policy.state != 'idle' or policy.pending is None:
                return
            if not self.client.server_is_ready():
                self.warn('server', 'NavigateToPose server unavailable; retaining only latest fresh goal')
                return
            token, goal = policy.begin_send()
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = goal.frame
            request.pose.header.stamp.sec, request.pose.header.stamp.nanosec = divmod(
                goal.stamp_ns, 1_000_000_000)
            p, q = request.pose.pose.position, request.pose.pose.orientation
            p.x, p.y, p.z = goal.position
            q.x, q.y, q.z, q.w = goal.quaternion
            self.get_logger().info(f'Sending task {token}')
            try:
                self.client.send_goal_async(request).add_done_callback(
                    lambda future, t=token: self.accepted(t, future))
            except Exception as exc:
                self.failed(token, 'send', exc)

        def failed(self, token, operation, exc):
            if token != self.policy.token:
                return
            self.policy.fault(token)
            self.get_logger().error(
                f'Task {token} {operation} failed: {exc}; dispatch blocked because remote '
                'state is uncertain. Verify/cancel remote goal before restarting bridge.')

        def accepted(self, token, future):
            try:
                handle = future.result()
                if not self.policy.accepted(token, handle.accepted):
                    return
                if not handle.accepted:
                    self.get_logger().warning(f'Task {token} rejected; not automatically retried')
                    return
                self.handle = handle
                self.get_logger().info(f'Task {token} accepted')
                handle.get_result_async().add_done_callback(
                    lambda result, t=token: self.result(t, result))
            except Exception as exc:
                self.failed(token, 'accept/result registration', exc)

        def canceled(self, token, future):
            if token != self.policy.token:
                return
            try:
                response = future.result()
                matched = any(bytes(info.goal_id.uuid) == bytes(self.handle.goal_id.uuid)
                              for info in response.goals_canceling)
                if response.return_code == 0 and matched:
                    self.get_logger().info(f'Task {token} cancellation acknowledged; still awaiting result')
                else:
                    self.get_logger().warning(
                        f'Task {token} cancellation not acknowledged (code={response.return_code}); '
                        'no replacement until terminal result, no cancellation retry flood')
            except Exception as exc:
                self.failed(token, 'cancel response', exc)

        def result(self, token, future):
            if token != self.policy.token:
                return
            try:
                response = future.result()
                names = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                         GoalStatus.STATUS_CANCELED: 'canceled',
                         GoalStatus.STATUS_ABORTED: 'aborted'}
                if response.status not in names:
                    raise RuntimeError(f'non-terminal result status {response.status}')
                self.get_logger().info(
                    f'Task {token} {names[response.status]}; result={response.result}')
                if self.policy.finish(token):
                    self.handle = None
            except Exception as exc:
                self.failed(token, 'result', exc)

        def stop(self):
            self.policy.stop()
            # A blocked result request may still leave a known handle to cancel.
            if self.policy.state == 'blocked' and self.handle is not None:
                try:
                    self.handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(f'Best-effort shutdown cancel failed: {exc}')
            self.tick()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor()
    running = True

    def stop_signal(signum, frame):
        nonlocal running
        running = False

    old_handlers = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = Adapter()
        executor.add_node(node)
        while running and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
                deadline = time.monotonic() + node.config['shutdown_timeout_sec']
                while node.policy.state != 'idle' and time.monotonic() < deadline and rclpy.ok():
                    executor.spin_once(timeout_sec=0.05)
                    node.tick()
            if node.policy.state != 'idle':
                node.get_logger().warning(
                    'Shutdown without confirmed terminal result; remote navigation may continue. '
                    'Use an independent robot stop/cancel mechanism.')
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
