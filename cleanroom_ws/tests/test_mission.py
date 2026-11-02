"""Real DDS query switching and one shared mission deadline; no simulator."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("mission_trial", SCRIPTS / "run_trial.py")
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


class MissionTests(unittest.TestCase):
    def exercise(self, seconds, delay):
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(driver, "ROOT", Path(temporary)),
        ):
            trial = driver.Trial(
                argparse.Namespace(
                    target="unused",
                    targets=["traffic light", "bus stop", "gas station"],
                    seconds=seconds,
                )
            )
            trial.listen()
            actor = Node("mission_test_sensors")
            queries = []
            current = [None, None]
            pubs = {
                "target": actor.create_publisher(
                    PoseStamped, "/spot1/raw_object_waypoint", 10
                ),
                "odom": actor.create_publisher(Odometry, "/spot1/odom", 10),
                "confirmation": actor.create_publisher(
                    String, "/spot1/object_confirmation", 10
                ),
                "event": actor.create_publisher(String, "/wildos/plan_status", 10),
            }

            def query(message):
                queries.append(message.data)
                current[:] = [message.data, time.monotonic()]

            actor.create_subscription(String, "/spot1/object_search/query", query, 10)

            def publish():
                q, started = current
                if q is None:
                    return
                # A success log alone must not switch goals.
                pubs["event"].publish(
                    String(data=json.dumps(dict(state="object_found", query=q)))
                )
                valid = time.monotonic() - started >= delay
                stamp = actor.get_clock().now().to_msg()
                target = PoseStamped()
                target.header.stamp, target.header.frame_id = stamp, "spot1/odom"
                target.pose.position.x, target.pose.position.z = 2.9, 58.0
                pubs["target"].publish(target)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.position.z = 52.0
                pubs["odom"].publish(odom)
                pubs["confirmation"].publish(
                    String(
                        data=json.dumps(
                            dict(
                                query=q if valid else "previous query",
                                frame="spot1/odom",
                                search_confirmed=True,
                                object_position=[2.9, 0.0, 58.0],
                                observation_stamp_ns=stamp.sec * 1_000_000_000
                                + stamp.nanosec,
                            )
                        )
                    )
                )

            actor.create_timer(0.02, publish)
            stop = threading.Event()
            executor = SingleThreadedExecutor()
            executor.add_node(actor)

            def receive():
                while not stop.is_set():
                    executor.spin_once(timeout_sec=0.02)

            thread = threading.Thread(target=receive)
            thread.start()
            try:
                end = time.monotonic() + 3
                while (
                    trial.query_pub.get_subscription_count() < 1
                    and time.monotonic() < end
                ):
                    time.sleep(0.02)
                trial.run_queries()
                elapsed = time.monotonic() - trial.query_at
                return queries, trial.stage_results, elapsed
            finally:
                stop.set()
                thread.join(timeout=2)
                executor.shutdown()
                actor.destroy_node()
                trial.node.destroy_node()
                rclpy.shutdown()

    def test_current_query_evidence_advances_three_goals_without_restart(self):
        queries, stages, elapsed = self.exercise(4, 0.3)
        self.assertEqual(queries, ["traffic light", "bus stop", "gas station"])
        self.assertTrue(all(s["passed"] for s in stages))
        self.assertEqual(len(stages), 3)
        self.assertTrue(all(s["ended_s"] - s["started_s"] >= 0.3 for s in stages))
        self.assertLess(elapsed, 4)

    def test_deadline_is_not_reset_and_failed_stage_never_skipped(self):
        queries, stages, elapsed = self.exercise(0.8, 0.5)
        self.assertEqual(queries, ["traffic light", "bus stop"])
        self.assertTrue(stages[0]["passed"])
        self.assertFalse(stages[1]["passed"])
        self.assertLess(elapsed, 1.1)


if __name__ == "__main__":
    unittest.main()
