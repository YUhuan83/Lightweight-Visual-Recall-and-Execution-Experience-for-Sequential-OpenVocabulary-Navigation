"""BeamNG transport stalls must not block ROS-side health and command code."""
import threading
import time
import unittest

from wildos_nav2_adapter.kinematic_rpc_worker import KinematicRpcWorker


class KinematicRpcWorkerTests(unittest.TestCase):
    def wait_until(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return predicate()

    def test_blocked_rpc_does_not_block_command_updates_or_health_reads(self):
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def exchange(linear, angular):
            calls.append((linear, angular))
            entered.set()
            release.wait(1.0)
            return dict(position=[0., 0., 0.], direction=[1., 0., 0.], up=[0., 0., 1.])

        worker = KinematicRpcWorker(exchange, lambda: None, interval_sec=.01)
        worker.start()
        try:
            self.assertTrue(entered.wait(.5))
            started = time.monotonic()
            worker.set_command(1.2, .3)
            health = worker.snapshot()
            self.assertLess(time.monotonic() - started, .1)
            self.assertFalse(health['telemetry_ok'])
            release.set()
            self.assertTrue(self.wait_until(lambda: worker.snapshot()['telemetry_ok']))
            self.assertTrue(self.wait_until(lambda: (1.2, .3) in calls))
        finally:
            release.set()
            self.assertTrue(worker.close())

    def test_rpc_failure_is_reported_and_success_recovers_health(self):
        calls = 0

        def exchange(_linear, _angular):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise TimeoutError('simulator RPC timed out')
            return dict(position=[1., 2., 3.])

        worker = KinematicRpcWorker(exchange, lambda: None, interval_sec=.01)
        worker.start()
        try:
            self.assertTrue(self.wait_until(lambda: worker.snapshot()['error'] is not None))
            failed = worker.snapshot()
            self.assertFalse(failed['telemetry_ok'])
            self.assertIn('simulator RPC timed out', failed['error'])
            time.sleep(.05)
            self.assertEqual(calls, 1, 'failed RPC must not reconnect in a tight retry loop')
            worker.retry()
            self.assertTrue(self.wait_until(lambda: worker.snapshot()['telemetry_ok']))
        finally:
            self.assertTrue(worker.close())

    def test_inactive_worker_makes_no_rpc_calls_until_activated(self):
        calls = []

        def exchange(linear, angular):
            calls.append((linear, angular))
            return dict(position=[0., 0., 0.])

        worker = KinematicRpcWorker(exchange, lambda: None, interval_sec=.01)
        worker.set_active(False)
        worker.start()
        try:
            time.sleep(.05)
            self.assertEqual(calls, [])
            self.assertFalse(worker.snapshot()['telemetry_ok'])
            worker.set_active(True)
            self.assertTrue(self.wait_until(lambda: len(calls) > 0))
            self.assertTrue(worker.snapshot()['telemetry_ok'])
        finally:
            self.assertTrue(worker.close())

    def test_close_serializes_final_stop_after_an_inflight_rpc(self):
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def exchange(linear, angular):
            calls.append(('exchange', linear, angular))
            entered.set()
            release.wait(1.0)
            return dict(position=[0., 0., 0.])

        def stop_rpc():
            calls.append(('stop',))

        worker = KinematicRpcWorker(exchange, stop_rpc, interval_sec=.01)
        worker.start()
        self.assertTrue(entered.wait(.5))
        closer = threading.Thread(target=worker.close)
        closer.start()
        release.set()
        closer.join(1.0)
        self.assertFalse(closer.is_alive())
        self.assertEqual(calls[-1], ('stop',))


if __name__ == '__main__':
    unittest.main()
