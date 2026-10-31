"""The native vehicle freshness callback must progress during graph work."""
import json
import threading
import time
import unittest
from unittest.mock import Mock
from rclpy.task import Future

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from wildos_nav2_adapter.input_quality import stamp_ns


class NativeStatusExecutorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def odometry_during_navigation_work(self, direction):
        bridge=GraphPathBridge();source=Node('odometry_executor_probe')
        for timer in list(bridge.timers):
            bridge.destroy_timer(timer)
        publisher=source.create_publisher(Odometry,'/spot1/odom',qos_profile_sensor_data)
        executor=MultiThreadedExecutor(num_threads=2)
        executor.add_node(bridge);executor.add_node(source)
        spinning=threading.Thread(target=executor.spin,daemon=True);spinning.start()
        stop=threading.Event();finished=threading.Event();entered=threading.Event()
        latest={};observed={};origin=time.monotonic()
        def publish():
            while not stop.is_set():
                msg=Odometry();msg.header.frame_id='spot1/odom'
                msg.header.stamp=source.get_clock().now().to_msg()
                msg.pose.pose.orientation.w=1.
                msg.pose.pose.position.x=direction*.6*(time.monotonic()-origin)
                latest['message']=msg
                publisher.publish(msg);stop.wait(.04)
        publishing=threading.Thread(target=publish,daemon=True);publishing.start()
        try:
            deadline=time.monotonic()+3.
            while bridge.odom is None and time.monotonic()<deadline:time.sleep(.01)
            self.assertIsNotNone(bridge.odom)
            def navigation_work():
                if entered.is_set():return
                entered.set();before=stamp_ns(bridge.odom)
                epoch=bridge.epoch
                time.sleep(.65)
                observed['stable_during_work']=stamp_ns(bridge.odom)==before
                consume=getattr(bridge,'consume_pending_odometry',None)
                if consume is not None:consume()
                observed['age_s']=(bridge.get_clock().now().nanoseconds-stamp_ns(bridge.odom))/1e9
                observed['position_error_m']=abs(bridge.odom.pose.pose.position.x-latest['message'].pose.pose.position.x)
                observed['epoch_unchanged']=bridge.epoch==epoch
                finished.set()
            bridge.create_timer(.02,navigation_work)
            self.assertTrue(finished.wait(3.))
            self.assertTrue(observed['stable_during_work'])
            self.assertTrue(observed['epoch_unchanged'])
            return observed
        finally:
            stop.set();publishing.join(timeout=2.)
            executor.shutdown();spinning.join(timeout=2.)
            source.destroy_node();bridge.destroy_node()

    def test_forward_odometry_is_fresh_at_navigation_callback_handoff(self):
        observed=self.odometry_during_navigation_work(1)
        self.assertLess(observed['age_s'],.2)

    def test_reverse_stop_pose_uses_current_odometry_after_navigation_work(self):
        observed=self.odometry_during_navigation_work(-1)
        self.assertLess(observed['position_error_m'],.06)

    def test_navigation_result_waits_for_state_group_and_checks_retired_epoch(self):
        bridge = GraphPathBridge()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(bridge)
        entered, release, finished = (threading.Event() for _ in range(3))
        callback_ran = threading.Event()
        handled = threading.Event()
        future = Future(executor=executor)
        epoch = bridge.epoch
        index, serial = bridge.candidate_index, 17
        bridge.active_plan_serial = serial
        handle = Mock(accepted=True)

        def consume(completed):
            callback_ran.set()
            bridge.on_plan_accepted(completed, epoch, index, serial)
            handled.set()

        bridge.add_navigation_done_callback(future, consume)

        def retire_query():
            if entered.is_set():
                return
            entered.set()
            release.wait(2.)
            bridge.epoch += 1
            finished.set()

        timer = bridge.create_timer(.05, retire_query)
        spinning = threading.Thread(target=executor.spin, daemon=True)
        spinning.start()
        try:
            self.assertTrue(entered.wait(2.))
            future.set_result(handle)
            # Future's task can run on worker 2. Its navigation callback must
            # wait for worker 1 to finish changing query/reset ownership.
            self.assertFalse(callback_ran.wait(.2))
            release.set()
            self.assertTrue(finished.wait(2.))
            self.assertTrue(handled.wait(2.))
            handle.cancel_goal_async.assert_called_once()
            handle.get_result_async.assert_not_called()
        finally:
            release.set()
            timer.cancel()
            executor.shutdown()
            spinning.join(timeout=2.)
            bridge.destroy_node()

    def test_native_receipt_proves_same_sequence_when_control_ack_lags_one_tick(self):
        bridge = GraphPathBridge()
        try:
            now_ns = time.time_ns()
            sender = '75a1dd23041e4f53b0b2dd4f12026f10'
            receipt = {
                'version': 'wildos_source_control_v1', 'clock': 'unix_ns',
                'sender_id': sender, 'generation': 7, 'sequence': 1385,
                'issued_wall_ns': now_ns - 1_000_000,
                'expires_wall_ns': now_ns + 250_000_000,
                'control': {'throttle': 0.0, 'brake': 1.0, 'steering': 0.0,
                            'parkingbrake': 0.0, 'clutch': 0.0, 'gear': 2},
                'applied_wall_ns': now_ns - 500_000, 'applied': True}
            bridge.carrier_model = 'ackermann'
            bridge.follow_enabled = True
            bridge.require_source_control = True
            bridge.native_status = {
                'wall_time': now_ns * 1e-9, 'speed': 0.0,
                'actuator_ok': True, 'telemetry_ok': True,
                'telemetry_age_sec': 0.0, 'control_acknowledged': True,
                'control_protocol': 'wildos_source_control_v1',
                'source_receipt': receipt}
            bridge.native_status_at = time.monotonic()
            bridge.control_status = {
                'control_protocol': 'wildos_source_control_v1',
                'source_ack_ready': False,
                'source_control': {'sender_id': sender, 'generation': 7,
                                   'sequence': 1385}}
            bridge.control_status_at = time.monotonic()
            self.assertIsNone(bridge.execution_transport_issue())
            bridge.control_status['source_control']['sequence'] += 1
            self.assertEqual(bridge.execution_transport_issue(),
                             'source_owner_unconfirmed')
        finally:
            bridge.destroy_node()

    def test_native_status_advances_while_default_group_is_blocked(self):
        bridge = GraphPathBridge()
        source = Node('native_status_executor_probe')
        publisher = source.create_publisher(String, '/wildos/native_control_status', 10)
        control_publisher = source.create_publisher(String, '/wildos/control_status', 10)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(bridge)
        executor.add_node(source)
        spinning = threading.Thread(target=executor.spin, daemon=True)
        spinning.start()
        stop_publish = threading.Event()
        blocked = threading.Event()
        timer_done = threading.Event()

        def publish_status():
            message = String()
            message.data = json.dumps({
                'wall_time': time.time(), 'speed': 0.0, 'actual': {},
                'wheel_contact': {'wall_time': time.time(), 'diagnosis': {}},
                'sent': {'brake': 1.0}, 'telemetry_age_sec': 0.02,
                'telemetry_ok': True, 'actuator_ok': True,
                'control_acknowledged': True})
            publisher.publish(message)
            control = String()
            control.data = json.dumps({
                'stamp_ns': bridge.get_clock().now().nanoseconds,
                'actuating': True, 'target_speed': 0.5, 'target_angular': 0.0,
                'source_control': {'sender_id': 'probe', 'generation': 1},
                'source_ack_ready': True})
            control_publisher.publish(control)

        def publish_loop():
            while not stop_publish.is_set():
                publish_status()
                stop_publish.wait(0.04)

        publishing = threading.Thread(target=publish_loop, daemon=True)
        publishing.start()
        try:
            deadline = time.monotonic() + 3.0
            while bridge.native_status_snapshot()[0] is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIsNotNone(bridge.native_status_snapshot()[0])
            deadline = time.monotonic() + 3.0
            while bridge.control_status_snapshot()[0] is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIsNotNone(bridge.control_status_snapshot()[0])

            def block_default_group():
                if timer_done.is_set():
                    return
                blocked.set()
                timer_done.set()
                time.sleep(0.8)

            bridge.create_timer(0.1, block_default_group)
            self.assertTrue(blocked.wait(2.0))
            before, _ = bridge.native_status_snapshot()
            control_before, _ = bridge.control_status_snapshot()
            time.sleep(0.5)
            after, received_at = bridge.native_status_snapshot()
            control_after, control_received_at = bridge.control_status_snapshot()
            self.assertGreater(after['wall_time'], before['wall_time'] + 0.3)
            self.assertLess(time.monotonic() - received_at, 0.25)
            self.assertGreater(control_after['stamp_ns'], control_before['stamp_ns'] + 300_000_000)
            self.assertLess(time.monotonic() - control_received_at, 0.25)
        finally:
            stop_publish.set()
            executor.shutdown()
            spinning.join(timeout=2.0)
            source.destroy_node()
            bridge.destroy_node()
