"""Start one clean stack; query timeout excludes startup and shutdown."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trial_audit import (
    independent_arrival_audit,
    motion_metrics,
    normalize_query,
    query_consumers_ready,
)

ROOT = Path(__file__).resolve().parents[1]


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Trial:
    def __init__(self, args):
        self.args = args
        self.queries = [
            normalize_query(q)
            for q in (getattr(args, "targets", None) or [args.target])
        ]
        self.stage_index = 0
        self.stage_at = None
        self.stage_first_target = None
        self.stage_results = []
        self.output = ROOT / "runs" / time.strftime("%Y%m%d_%H%M%S")
        self.output.mkdir(parents=True)
        self.children = []
        self.query_at = None
        self.query_started_ns = None
        self.samples = []
        self.events = []
        self.first_target = None
        self.latest_image = None
        self.confirmation_image = None
        self.latest_odom = None
        self.target = None
        self.confirmation = None
        self.confirmation_received_ns = None
        self.object_found_event = False
        self.arrival_audit = {"verified": False, "reason": "query_not_started"}
        self.closed = False

    @property
    def current_query(self):
        return self.queries[self.stage_index]

    def spawn(self, role, argv, env=None, cwd=None):
        process = subprocess.Popen(
            argv,
            env=env,
            cwd=cwd,
            stdout=(self.output / (role + ".log")).open("w"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.children.append((role, process))
        return process

    def wait(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            failed = [
                (role, p.returncode)
                for role, p in self.children
                if p.poll() not in (None, 0)
            ]
            if failed:
                raise RuntimeError("Startup child failed: " + repr(failed))
            time.sleep(0.1)
        raise TimeoutError("Readiness timeout")

    def on_odom(self, message):
        if self.query_at is not None:
            speed = message.twist.twist.linear
            point = message.pose.pose.position
            odom_xy = (float(point.x), float(point.y))
            odom_stamp = stamp_ns(message.header.stamp)
            self.latest_odom = (odom_xy, odom_stamp)
            self.samples.append(
                (
                    time.monotonic() - self.query_at,
                    float(speed.x),
                    float(speed.y),
                    float(point.x),
                    float(point.y),
                )
            )
            self.update_arrival_audit()

    def on_event(self, message):
        event = json.loads(message.data)
        event["search_elapsed_s"] = (
            time.monotonic() - self.query_at if self.query_at else None
        )
        event["stage_index"] = self.stage_index + 1
        event["stage_elapsed_s"] = (
            time.monotonic() - self.stage_at if self.stage_at else None
        )
        self.events.append(event)
        if (
            event.get("state") == "object_found"
            and normalize_query(event.get("query", "")) == self.current_query
        ):
            self.object_found_event = True
        with (self.output / "events.jsonl").open("a") as log:
            log.write(json.dumps(event) + "\n")

    def on_target(self, message):
        if self.query_at is None or message.header.frame_id != "spot1/odom":
            return
        stamp = stamp_ns(message.header.stamp)
        if stamp < self.query_started_ns:
            return
        position = message.pose.position
        self.target = (
            (float(position.x), float(position.y), float(position.z)),
            stamp,
        )
        if self.first_target is None:
            self.first_target = time.monotonic() - self.query_at
        if self.stage_first_target is None:
            self.stage_first_target = time.monotonic() - self.stage_at
        with (self.output / "target_updates.jsonl").open("a") as log:
            log.write(
                json.dumps(
                    dict(
                        query=self.current_query,
                        stage=self.stage_index + 1,
                        stamp_ns=stamp,
                        xyz=self.target[0],
                    )
                )
                + "\n"
            )
        self.update_arrival_audit()

    def on_confirmation(self, message):
        if self.query_started_ns is None:
            return
        try:
            data = json.loads(message.data)
            now_ns = self.node.get_clock().now().nanoseconds
            observed_ns = int(data.get("observation_stamp_ns", 0))
            if (
                normalize_query(data.get("query", "")) != self.current_query
                or data.get("frame") != "spot1/odom"
                or data.get("search_confirmed") is not True
                or not self.query_started_ns <= observed_ns <= now_ns
                or now_ns - observed_ns > 5_000_000_000
            ):
                return
        except (TypeError, ValueError):
            return
        self.confirmation = data
        self.confirmation_received_ns = now_ns
        with (self.output / "confirmation.jsonl").open("a") as log:
            log.write(json.dumps(data) + "\n")
        self.update_arrival_audit()

    def update_arrival_audit(self):
        if self.query_started_ns is None:
            return
        odom = self.latest_odom
        target = self.target
        self.arrival_audit = independent_arrival_audit(
            odom[0] if odom else None,
            odom[1] if odom else 0,
            target[0] if target else None,
            target[1] if target else 0,
            self.confirmation,
            self.current_query,
            self.query_started_ns,
            self.node.get_clock().now().nanoseconds,
            self.confirmation_received_ns,
        )

    def listen(self):
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Image

        rclpy.init()
        self.node = Node("cleanroom_trial")
        self.node.create_subscription(Odometry, "/spot1/odom", self.on_odom, 10)
        self.node.create_subscription(String, "/wildos/plan_status", self.on_event, 10)
        self.node.create_subscription(
            PoseStamped, "/spot1/raw_object_waypoint", self.on_target, 10
        )
        self.node.create_subscription(
            String, "/spot1/object_confirmation", self.on_confirmation, 10
        )
        self.node.create_subscription(
            Image, "/spot1/realsense/front/color/image_raw", self.on_image, 1
        )
        self.stop_client = self.node.create_client(Trigger, "/wildos/stop")
        self.query_pub = self.node.create_publisher(
            String, "/spot1/object_search/query", 10
        )

    def run_queries(self):
        """One fixed mission deadline; advance only after verified arrival."""
        self.query_at = time.monotonic()
        deadline = self.query_at + self.args.seconds
        for index, query in enumerate(self.queries):
            self.stage_index = index
            self.stage_at = time.monotonic()
            self.query_started_ns = self.node.get_clock().now().nanoseconds
            self.target = self.confirmation = self.confirmation_image = None
            self.confirmation_received_ns = None
            self.stage_first_target = None
            self.object_found_event = False
            self.arrival_audit = {"verified": False, "reason": "query_not_observed"}
            self.query_pub.publish(String(data=query))
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                self.update_arrival_audit()
                if self.object_found_event and self.arrival_audit["verified"]:
                    break
            passed = bool(
                time.monotonic() <= deadline
                and self.object_found_event
                and self.arrival_audit["verified"]
            )
            self.stage_results.append(
                dict(
                    index=index + 1,
                    query=query,
                    passed=passed,
                    started_s=self.stage_at - self.query_at,
                    ended_s=time.monotonic() - self.query_at,
                    first_target_s=self.stage_first_target,
                    object_found_event=self.object_found_event,
                    target_xyz=self.target[0] if self.target else None,
                    arrival_audit=dict(self.arrival_audit),
                    reason="verified_arrival" if passed else "mission_deadline",
                )
            )
            self.save_images(f"stage_{index + 1:02d}_")
            (self.output / "stages.json").write_text(
                json.dumps(self.stage_results, indent=2)
            )
            if not passed or time.monotonic() >= deadline:
                break

    def run(self):
        self.listen()
        started = time.monotonic()
        summary = dict(
            passed=False, query=self.queries[0], queries=self.queries, errors=[]
        )
        try:
            if not self.args.attach:
                home = Path(os.environ["BEAMNG_HOME"])
                user = os.environ["BEAMNG_USER"]
                display = os.environ.get("DISPLAY", ":148")
                if not Path("/tmp/.X11-unix/X" + display.lstrip(":")).exists():
                    self.spawn(
                        "display",
                        [
                            "Xvfb",
                            display,
                            "-screen",
                            "0",
                            "960x640x24",
                            "-nolisten",
                            "tcp",
                        ],
                    )
                os.environ["DISPLAY"] = display
                self.spawn(
                    "engine",
                    [
                        str(home / "BinLinux/BeamNG.tech.x64"),
                        "-headless",
                        "-nosfx",
                        "-userpath",
                        user,
                        "-tcom",
                        "-tport",
                        str(self.args.port),
                        "-tcom-listen-ip",
                        self.args.host,
                        "-nosteam",
                    ],
                    cwd=home,
                )
                self.wait(self.listening, 90)
                subprocess.run(
                    [
                        sys.executable,
                        str(ROOT / "scripts/wildos_console_scene.py"),
                        "--home",
                        str(home),
                        "--user",
                        user,
                        "--host",
                        self.args.host,
                        "--port",
                        str(self.args.port),
                        "--output",
                        str(self.output / "scene.json"),
                    ],
                    check=True,
                    timeout=150,
                )
            self.spawn(
                "sensors",
                [
                    sys.executable,
                    str(ROOT / "scripts/attach_current_vehicle_sensors.py"),
                    "--rig",
                    str(ROOT / "config/sensors.json"),
                    "--output",
                    str(self.output / "sensors.json"),
                    "--host",
                    self.args.host,
                    "--port",
                    str(self.args.port),
                ],
            )
            self.wait(lambda: (self.output / "sensors.json").exists(), 60)
            vehicle_port = json.loads((self.output / "sensors.json").read_text())[
                "vehicle_port"
            ]
            self.spawn(
                "stack",
                [
                    "ros2",
                    "launch",
                    "wildos_nav2_adapter",
                    "cleanroom.launch.py",
                    "host:=" + self.args.host,
                    "port:=" + str(self.args.port),
                    "vehicle_port:=" + str(vehicle_port),
                ],
            )
            vision_python = os.environ["VISION_PYTHON"]
            self.spawn(
                "triangulation",
                [
                    vision_python,
                    "-m",
                    "visual_navigation.explorfm_triangulation.obj_mask_triangulation",
                    "--config",
                    str(ROOT / "config/beamng_triangulation.yaml"),
                ],
            )
            vision_config = ROOT / "config/beamng_wildos.yaml"
            if os.environ.get("CKPT_ROOT"):
                import yaml

                data = yaml.safe_load(vision_config.read_text())
                data.update(
                    ckpt_root=os.environ["CKPT_ROOT"],
                    adaptor_ckpt_path=os.environ["CKPT_ROOT"] + "/siglip2",
                )
                vision_config = self.output / "vision.yaml"
                vision_config.write_text(yaml.safe_dump(data))
            self.spawn(
                "vision",
                [
                    vision_python,
                    "-m",
                    "visual_navigation.wildos.nav",
                    "--config",
                    str(vision_config),
                    "--do_object_search",
                    "true",
                    "--memory_in_process_only",
                ],
            )
            deadline = time.monotonic() + 300
            while not query_consumers_ready(
                info.node_name
                for info in self.node.get_subscriptions_info_by_topic(
                    "/spot1/object_search/query"
                )
            ):
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        "Vision/triangulation/bridge query subscriptions not ready"
                    )
                rclpy.spin_once(self.node, timeout_sec=0.1)
                for role, process in self.children:
                    if process.poll() is not None:
                        raise RuntimeError(
                            f"{role} exited before query: {process.returncode}"
                        )
            summary["startup_s"] = time.monotonic() - started
            self.run_queries()
            summary["search_s"] = time.monotonic() - self.query_at
        except Exception as error:
            summary["errors"].append(repr(error))
        finally:
            try:
                self.shutdown()
            except Exception as error:
                summary["errors"].append("shutdown: " + repr(error))
            summary["closed"] = self.closed
            summary["first_target_s"] = self.first_target
            summary["object_found_event"] = self.object_found_event
            summary["independent_visual_arrival_verified"] = self.arrival_audit[
                "verified"
            ]
            summary["arrival_audit"] = self.arrival_audit
            summary["stages"] = self.stage_results
            summary["completed_goals"] = sum(
                stage["passed"] for stage in self.stage_results
            )
            summary["remaining_queries"] = self.queries[summary["completed_goals"] :]
            search_s = summary.get(
                "search_s", time.monotonic() - self.query_at if self.query_at else 0.0
            )
            exploration_s = min(
                search_s,
                self.first_target if self.first_target is not None else search_s,
            )
            summary["exploration_motion"] = motion_metrics(self.samples, exploration_s)
            summary["passed"] = bool(
                len(self.stage_results) == len(self.queries)
                and all(stage["passed"] for stage in self.stage_results)
                and self.closed
                and not summary["errors"]
            )
            (self.output / "odom.json").write_text(json.dumps(self.samples))
            (self.output / "result.json").write_text(json.dumps(summary, indent=2))
            print(json.dumps(dict(output=str(self.output), **summary), indent=2))
            self.node.destroy_node()
            rclpy.shutdown()

    def listening(self):
        try:
            with socket.create_connection(
                (self.args.host, self.args.port), timeout=0.2
            ):
                return True
        except OSError:
            return False

    def on_image(self, message):
        self.latest_image = message
        if self.confirmation is not None and self.confirmation_image is None:
            self.confirmation_image = message

    def shutdown(self):
        try:
            if self.stop_client.service_is_ready():
                future = self.stop_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            self.save_images("")
        finally:
            for _, process in reversed(self.children):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for _, process in reversed(self.children):
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
            self.closed = all(
                process.poll() is not None for _, process in self.children
            )

    def save_images(self, prefix):
        import cv2
        from cv_bridge import CvBridge

        bridge = CvBridge()
        for name, message in (
            ("last_rgb.png", self.latest_image),
            ("confirmation_rgb.png", self.confirmation_image),
        ):
            if message is not None:
                image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                if not cv2.imwrite(str(self.output / (prefix + name)), image):
                    raise OSError(f"Failed to write {prefix + name}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("target", nargs="?", default="green residential house")
    parser.add_argument(
        "--targets",
        nargs="+",
        help="Ordered goals sharing one map, process and total deadline",
    )
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25252)
    parser.add_argument(
        "--attach", action="store_true", help="Use the map already loaded by the user"
    )
    Trial(parser.parse_args()).run()


if __name__ == "__main__":
    main()
