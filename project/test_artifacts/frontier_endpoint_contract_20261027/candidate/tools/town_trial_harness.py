"""Shared trial callbacks; importing this module opens no simulator or files."""
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
from beamng_msgs.msg import StateSensor
from object_search_msgs.msg import ObjectMaskWithTf
from sensor_msgs.msg import Image
from std_msgs.msg import String

from wildos_nav2_adapter.arrival_criteria import (
    STANDOFF_ARRIVAL_TOLERANCE_M,
    arrival_status_matches,
    mask_matches_confirmation,
    matched_arrival_evidence,
)
from wildos_nav2_adapter.graph_path_bridge import xy, yaw
from wildos_nav2_adapter.test_progress import ManeuverProgress



class Harness(Node):
    def __init__(self, output_dir, vision_pid, carrier_model="ideal_point",
                 record_video=False, video_sink=None, frame_callback=None):
        super().__init__("green_house_station_arrival_test")
        self.out = Path(output_dir)
        self.vision_pid = int(vision_pid)
        if carrier_model not in ("ideal_point", "ackermann") or self.vision_pid < 1:
            raise ValueError("Valid carrier model and vision PID required")
        self.carrier_model = carrier_model
        self.expected_planner = "CarLike" if carrier_model == "ackermann" else "GridBased"
        self.expected_radius = 8.0 if carrier_model == "ackermann" else 0.0
        self.phase = "setup"
        self.query = "green residential house"
        self.query_started_ns = 0
        self.state = {}
        self.events = []
        self.masks = {}
        self.mask_history = deque(maxlen=128)
        self.confirmation_history = deque(maxlen=128)
        self.views = {}
        # Cover the existing five-second arrival evidence window, bounded.
        self.frames = deque(maxlen=64)
        self.mask_images = deque(maxlen=64)
        self.frame_callback = frame_callback
        self.latest_frame = None
        self.latest_frame_stamp = None
        self.latest_frame_stamp_ns = None
        self.latest_overlay = None
        self.latest_overlay_stamp = None
        self.latest_overlay_stamp_ns = None
        self.latest_overlay_query = None
        self.video = video_sink
        if self.video is None and record_video:
            self.video = cv2.VideoWriter(
                str(self.out / "front_camera.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 5, (960, 640))
            if not self.video.isOpened():
                raise RuntimeError("Failed to open front_camera.mp4 writer")
        self.log = (self.out / "events.jsonl").open("w", encoding="utf-8")
        self.native_log = (self.out / "native_samples.jsonl").open("w", encoding="utf-8")
        self.sensor_timing_log = (self.out / "sensor_timing.jsonl").open("w", encoding="utf-8")
        self.sensor_timing_records = 0
        self.started_at = time.monotonic()
        self.last_native_write = 0.0
        self.native_gap_started = None
        self.native_fault_reported = False
        self.actuator_bad_since = None
        self.last_status = None
        self.arrival_statuses = deque(maxlen=64)
        self.waypoint = None
        self.confirmation = None
        self.query_pub = self.create_publisher(String, "/spot1/object_search/query", 10)
        self.cmd_pub = self.create_publisher(Twist, "/wildos/cmd_vel", 1)
        self.create_subscription(String, "/wildos/native_control_status", self.native, 10)
        self.create_subscription(String, "/wildos/plan_status", self.status, 10)
        self.create_subscription(String, "/spot1/measurement_timing", self.measurement_timing, 10)
        self.create_subscription(String, "/spot1/remembered_object_candidates", self.candidates, 10)
        self.create_subscription(String, "/wildos/memory_selection", self.selection, 10)
        self.create_subscription(PoseStamped, "/spot1/imgnav_waypoint", self.goal, 10)
        self.create_subscription(String, "/spot1/object_confirmation", self.confirmed, 10)
        self.create_subscription(ObjectMaskWithTf, "/spot1/object_mask", self.mask, 10)
        self.create_subscription(Image, "/spot1/realsense/front/color/image_raw", self.camera, 1)
        self.create_subscription(Route, "/wildos/planned_path", self.path, 10)
        self.create_subscription(
            Odometry, "/spot1/odom", self.odom, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", lambda msg: self.state.update(grid=msg), 10)
        self.create_timer(0.2, self.record_camera)

    def event(self, kind, **details):
        item = {"elapsed": time.monotonic() - self.started_at, "monotonic_ns": time.monotonic_ns(), "phase": self.phase,
                "query": self.query, "kind": kind, **details}
        self.events.append(item)
        self.log.write(json.dumps(item) + "\n")
        self.log.flush()
        print(json.dumps(item), flush=True)

    def record_timing(self, topic, header_stamp_ns=None, stamp_clock="ros", **details):
        received_ros_ns = self.get_clock().now().nanoseconds
        item = {"topic": topic, "stamp_clock": stamp_clock,
                "header_stamp_ns": header_stamp_ns,
                "received_ros_ns": received_ros_ns,
                "received_wall_ns": time.time_ns(),
                "received_monotonic_ns": time.monotonic_ns(), **details}
        if stamp_clock == "ros" and header_stamp_ns is not None:
            item["ros_header_age_ns"] = received_ros_ns - int(header_stamp_ns)
        self.sensor_timing_log.write(json.dumps(item, separators=(",", ":")) + "\n")
        self.sensor_timing_records += 1
        if self.sensor_timing_records % 64 == 0:
            self.sensor_timing_log.flush()

    def measurement_timing(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            self.record_timing("/spot1/measurement_timing", status="invalid_json")
            return
        stamp_ns = data.get("mapped_stamp_ns", data.get("mask_stamp_ns"))
        stamp_clock = data.get(
            "output_clock", data.get("clock", "ros" if stamp_ns is not None
                                     else "beamng_source"))
        self.record_timing("/spot1/measurement_timing", stamp_ns,
                           stamp_clock, adapter_record=data)

    def odom(self, msg):
        self.state["odom"] = msg
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.record_timing("/spot1/odom", stamp, frame_id=msg.header.frame_id,
                           child_frame_id=msg.child_frame_id)

    def native(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            self.event("native_telemetry_invalid_json")
            return
        received_at = time.monotonic()
        telemetry_age = data.get("telemetry_age_sec")
        try:
            telemetry_age = float(telemetry_age)
        except (TypeError, ValueError):
            telemetry_age = None
        if telemetry_age is not None and (not math.isfinite(telemetry_age) or telemetry_age < 0.):
            telemetry_age = None
        self.state["native"] = data
        # An inactive RPC worker reports unknown age. Preserve the last known
        # sample time instead of inventing a timeout age for that heartbeat.
        if telemetry_age is not None:
            self.state["native_at"] = received_at - telemetry_age
        else:
            self.state.setdefault("native_at", self.started_at)
        self.record_timing(
            "/wildos/native_control_status", data.get("state_stamp_ns"),
            "beamng_source", payload_wall_time=data.get("wall_time"),
            rpc_wall_time=data.get("rpc_wall_time"),
            telemetry_age_sec=telemetry_age,
            carrier_model=data.get("carrier_model"),
            beamng_carrier_model=data.get("beamng_carrier_model"),
            beamng_ros_yaw_to_cluster_sign=data.get("beamng_ros_yaw_to_cluster_sign"),
            beamng_rotation_convention=data.get("beamng_rotation_convention"),
            beamng_minimum_turning_radius_m=data.get("beamng_minimum_turning_radius_m"))
        if received_at - self.last_native_write >= 0.1:
            self.native_log.write(json.dumps({"phase": self.phase, **self.state["native"]}) + "\n")
            self.native_log.flush()
            self.last_native_write = received_at
        if data.get("actuator_ok") is False and not self.native_fault_reported:
            self.native_fault_reported = True
            self.event("native_actuator_fault", error=data.get("actuator_error") or
                       data.get("control_error"))

    def status(self, msg):
        data = json.loads(msg.data)
        self.record_timing("/wildos/plan_status", data.get("stamp_ns"),
                           schema_version=data.get("schema_version"),
                           status=data.get("status"), query=data.get("query"),
                           carrier_model=data.get("carrier_model"),
                           minimum_turning_radius_m=data.get("minimum_turning_radius_m"),
                           planner_id=data.get("planner_id"))
        profile_fields = ("schema_version", "carrier_model",
                          "minimum_turning_radius_m", "planner_id")
        # A freshly restarted bridge may first emit a legacy, metadata-free
        # idle heartbeat. Validate any status that carries a runtime profile;
        # the query handshake below separately requires the complete profile.
        if (any(field in data for field in profile_fields)
                and (data.get("schema_version") != 2
                     or data.get("carrier_model") != self.carrier_model
                     or data.get("minimum_turning_radius_m") != self.expected_radius
                     or data.get("planner_id") != self.expected_planner)):
            self.state["runtime_profile_error"] = {
                "source": "plan_status", "carrier_model": data.get("carrier_model"),
                "minimum_turning_radius_m": data.get("minimum_turning_radius_m"),
                "planner_id": data.get("planner_id"), "schema_version": data.get("schema_version")}
            self.event("runtime_profile_mismatch", **self.state["runtime_profile_error"])
        if data.get("status") == "object_found":
            self.arrival_statuses.append({"status": data, "received_at": time.monotonic()})
        if self.query == "gas station" and data.get("source") == "memory_via_graphnav":
            if data.get("status") in ("planned", "following") and self.state.get("path"):
                self.state.setdefault("first_memory_path_at", time.monotonic())
        if data != self.last_status:
            self.event("plan_status", **data)
            self.last_status = data

    def candidates(self, msg):
        data = json.loads(msg.data)
        self.views[data["query"]] = data["views"]
        if (data.get("query") == self.query
                and data.get("issued_at_ns", 0) >= self.query_started_ns
                and data.get("query_started_ns", 0) >= self.query_started_ns):
            self.state["vision_query_ack"] = data
        self.event("memory_candidates", payload=data)

    def selection(self, msg):
        data = json.loads(msg.data)
        self.state["selection"] = data
        self.event("memory_selection", payload=data)
        if (self.query == "gas station"
                and data.get("status") == "memory_candidate_selected"
                and self.state.get("first_memory_selection_at") is None):
            self.state["first_memory_selection_at"] = time.monotonic()

    def goal(self, msg):
        self.waypoint = {"point": xy(msg.pose), "at": time.monotonic(), "query": self.query}
        self.event("object_standoff", point=self.waypoint["point"])

    def confirmed(self, msg):
        data = json.loads(msg.data)
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = data.get("observation_stamp_ns", 0)
        if (data.get("query") != self.query or not isinstance(stamp_ns, int)
                or stamp_ns < self.query_started_ns or not 0 <= now_ns - stamp_ns <= 5 * 10**9):
            return
        if (data.get('localization_valid') is not True
                or (data.get('quality') or {}).get('association_validated') is not True):
            return
        self.confirmation = dict(data, received_at=time.monotonic())
        self.confirmation_history.append(self.confirmation)
        self.event("object_confirmation", payload=data)

    def path(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.record_timing("/wildos/planned_path", stamp,
                           frame_id=msg.header.frame_id, pose_count=len(msg.poses))
        points = [xy(pose.pose) for pose in msg.poses]
        self.state["path"] = points
        if self.query == "gas station" and len(points) >= 2:
            status = self.last_status or {}
            if status.get("source") == "memory_via_graphnav" and status.get("status") in ("planned", "following"):
                self.state.setdefault("first_memory_path_at", time.monotonic())
        self.event("planned_path", poses=len(points), points=points,
                   headings=[yaw(pose.pose) for pose in msg.poses],
                   length=sum(math.dist(a, b) for a, b in zip(points, points[1:])))

    def camera(self, msg):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.record_timing("/spot1/realsense/front/color/image_raw", stamp_ns,
                           frame_id=msg.header.frame_id, width=msg.width,
                           height=msg.height, encoding=msg.encoding, step=msg.step)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        frame = cv2.cvtColor(raw.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames.append((stamp_ns, frame.copy()))
        self.latest_frame = frame
        self.latest_frame_stamp = stamp
        self.latest_frame_stamp_ns = stamp_ns
        self.latest_frame_id = msg.header.frame_id

    def record_camera(self):
        if (self.video is not None or self.frame_callback is not None) and self.latest_frame is not None:
            frame = self.latest_frame
            if (self.latest_overlay is not None and self.latest_overlay_query == self.query
                    and self.latest_frame_stamp is not None
                    and self.latest_overlay_stamp is not None
                    and self.latest_frame_stamp_ns == self.latest_overlay_stamp_ns):
                frame = self.latest_overlay
            frame = cv2.resize(frame, (960, 640))
            if self.frame_callback is not None:
                self.frame_callback(frame, self.latest_frame_stamp_ns)
            if self.video is not None:
                self.video.write(frame)

    def mask(self, msg):
        if msg.query != self.query:
            return
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns < self.query_started_ns or not 0 <= now_ns - stamp_ns <= 5 * 10**9:
            return
        previous = self.masks.get(msg.query, {})
        if stamp_ns <= previous.get("observation_stamp_ns", 0):
            return
        pixels = int(sum(msg.object_mask.data))
        current = {"at": time.monotonic(), "pixels": pixels,
                   "count": previous.get("count", 0) + 1, "observation_stamp_ns": stamp_ns}
        current["query"] = msg.query
        self.masks[msg.query] = current
        self.mask_history.append(current)
        sizes = [dim.size for dim in msg.object_mask.layout.dim]
        mask = np.asarray(msg.object_mask.data, dtype=np.uint8).reshape(sizes)[0, 0] > 0
        self.mask_images.append((stamp_ns, mask.shape, np.packbits(mask)))
        matches = [frame for source_stamp, frame in self.frames if source_stamp == stamp_ns]
        if matches:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame = matches[-1]
            if frame.shape[:2] != mask.shape:
                self.event('mask_frame_shape_mismatch', stamp_ns=stamp_ns)
                return
            overlay = frame.copy()
            overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)
            self.latest_overlay = overlay
            self.latest_overlay_stamp = stamp
            self.latest_overlay_stamp_ns = stamp_ns
            self.latest_overlay_query = msg.query

    def save_failure_image(self):
        """Keep one raw diagnostic frame, including its age; this is not arrival evidence."""
        if self.latest_frame is None:
            return dict(available=False, reason='no_camera_frame')
        path = self.out / 'failure_last_rgb.png'
        ok, encoded = cv2.imencode('.png', self.latest_frame)
        if not ok:
            raise RuntimeError('Failed to encode failure snapshot')
        path.write_bytes(encoded.tobytes())
        return dict(available=True, rgb=str(path), query=self.query,
                    observation_stamp_ns=self.latest_frame_stamp_ns,
                    frame_id=self.latest_frame_id,
                    source_age_ros_sec=(self.get_clock().now().nanoseconds
                                        - self.latest_frame_stamp_ns) / 1e9,
                    is_arrival_evidence=False)

    def save_target_images(self, query):
        confirmation = self.confirmation or {}
        if confirmation.get('query') != query:
            raise RuntimeError('No matched arrival confirmation for snapshot')
        stamp_ns = confirmation.get('observation_stamp_ns')
        frames = [f for stamp, f in self.frames if stamp == stamp_ns]
        masks = [(shape, bits) for stamp, shape, bits in self.mask_images if stamp == stamp_ns]
        if not frames or not masks:
            raise RuntimeError('Exact arrival RGB/mask source frame is unavailable')
        frame = frames[-1]
        shape, bits = masks[-1]
        if tuple(frame.shape[:2]) != tuple(shape):
            raise RuntimeError('Arrival RGB/mask shape mismatch')
        mask = np.unpackbits(bits, count=int(np.prod(shape))).reshape(shape).astype(bool)
        overlay = frame.copy()
        overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)
        paths = {}
        for kind, image in [('rgb', frame), ('mask', overlay)]:
            # User-entered target text is data, never a directory component.
            stem = query.replace(' ', '_')
            if (len(stem) > 100 or not all(c.isalnum() or c in '_-' for c in stem)):
                stem = 'target_' + hashlib.sha256(query.encode('utf8')).hexdigest()[:16]
            path = self.out / f"{stem}_arrival_{kind}.png"
            ok, encoded = cv2.imencode('.png', image)
            if not ok:
                raise RuntimeError('Failed to encode arrival snapshot')
            path.write_bytes(encoded.tobytes())
            paths[kind] = str(path)
        return dict(observation_stamp_ns=stamp_ns, **paths)

    def set_bool(self, remote, name, value):
        if remote == "/wildos_cmd_vel_to_control" and self.carrier_model == "ideal_point":
            remote = "/wildos_kinematic_vehicle_control"
        client = self.create_client(SetParameters, remote + "/set_parameters")
        if not client.wait_for_service(timeout_sec=4):
            raise RuntimeError("Missing parameter service: " + remote)
        parameter = Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=value))
        future = client.call_async(SetParameters.Request(parameters=[parameter]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        self.destroy_client(client)
        if not future.done():
            raise RuntimeError("Parameter update timed out: " + remote + "/" + name)
        if future.exception():
            raise RuntimeError("Parameter update failed: " + str(future.exception()))
        results = future.result().results
        if len(results) != 1 or not all(result.successful for result in results):
            raise RuntimeError("Parameter update rejected: " + name + ": "
                               + str([result.reason for result in results]))

    def clear_costmaps(self):
        for prefix in ("local", "global"):
            client = self.create_client(ClearEntireCostmap,
                f"/{prefix}_costmap/clear_entirely_{prefix}_costmap")
            if not client.wait_for_service(timeout_sec=4):
                raise RuntimeError("Missing costmap clearing service: " + prefix)
            future = client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=8)
            self.destroy_client(client)
            if not future.done():
                raise RuntimeError("Costmap clear timed out: " + prefix)

    def spin(self, seconds, done=lambda: False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if done():
                return True
        return False

    def stop(self):
        self.cmd_pub.publish(Twist())
        errors = []
        # Close actuation even if canceling the route owner fails.
        for owner, name in [("/wildos_cmd_vel_to_control", "enable"),
                            ("/wildos_graph_path_bridge", "follow_path")]:
            try:
                self.set_bool(owner, name, False)
            except Exception as exc:
                errors.append(f'{owner}/{name}: {exc}')
        stopped = self.spin(3, lambda: (
            self.state.get("native", {}).get("speed", 99) < 0.1
            and time.monotonic() - self.state.get("native_at", 0.) <= .5))
        if errors:
            raise RuntimeError('Stop gate failure: ' + '; '.join(errors))
        if not stopped:
            raise RuntimeError('No fresh stationary vehicle telemetry after closing controls')
        return stopped

    def change_query(self, query):
        # Model construction precedes subscription creation. A published command
        # is not evidence that this named vision node has received the target.
        ready = lambda: any(info.node_name == "wildos" for info in
            self.get_subscriptions_info_by_topic("/spot1/object_search/query"))
        if not self.spin(90.0, ready):
            raise RuntimeError("Vision query subscriber did not become ready")
        self.state.pop("vision_query_ack", None)
        self.query = query
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.waypoint = None
        self.confirmation = None
        self.confirmation_history.clear()
        self.mask_history.clear()
        self.mask_images.clear()
        self.arrival_statuses.clear()
        self.masks.pop(query, None)
        self.latest_overlay = None
        self.latest_overlay_stamp = None
        self.latest_overlay_stamp_ns = None
        self.latest_overlay_query = None
        self.state.pop("selection", None)
        self.state.pop("path", None)
        if query == "gas station":
            self.state.pop("first_memory_selection_at", None)
            self.state.pop("first_memory_path_at", None)
        self.event("query_sent")
        self.query_pub.publish(String(data=query))
        if not self.spin(15.0, lambda: "vision_query_ack" in self.state):
            raise RuntimeError("Vision did not acknowledge target: " + query)
        self.event("vision_query_accepted", payload=self.state["vision_query_ack"])

    def arrival_evidence(self, query, minimum_mask_count):
        item = matched_arrival_evidence(
            list(self.arrival_statuses), list(self.confirmation_history),
            list(self.mask_history), query, self.query_started_ns,
            self.get_clock().now().nanoseconds, time.monotonic(), minimum_mask_count)
        if item is not None:
            # Use the paired observation in the report. Later callbacks must
            # not replace the evidence accepted by this arrival event.
            self.confirmation = item['confirmation']
            self.masks[query] = item['mask']
        return item

    def watch(self):
        now = time.monotonic()
        native_age = now - self.state.get("native_at", 0.0)
        if self.state.get("runtime_profile_error"):
            raise RuntimeError(self.carrier_model + " runtime profile mismatch: " +
                               json.dumps(self.state["runtime_profile_error"]))
        native = self.state.get("native")
        if (native and native.get("actuator_ok") is False
                and (native.get("actuator_error") or native.get("control_error"))):
            if self.actuator_bad_since is None:
                self.actuator_bad_since = now
                self.event("native_actuator_unhealthy_started", age_s=native_age,
                           actuator_error=native.get("actuator_error"),
                           control_error=native.get("control_error"))
            elif now - self.actuator_bad_since > 0.4:
                raise RuntimeError("Native actuator unhealthy for over 0.4s: " + str(
                    native.get("actuator_error") or native.get("control_error")))
        elif self.actuator_bad_since is not None:
            self.event("native_actuator_recovered",
                       duration_s=now - self.actuator_bad_since)
            self.actuator_bad_since = None
        native_age = now - self.state.get("native_at", 0.0)
        if native_age > 0.75:
            if self.native_gap_started is None:
                self.native_gap_started = now
                self.event("native_telemetry_gap_started", age_s=native_age,
                           abort_after_s=3.0)
            if native_age > 3.0:
                self.event("native_telemetry_stalled", age_s=native_age)
                raise RuntimeError("Native vehicle telemetry stalled for 3 seconds")
        elif self.native_gap_started is not None:
            self.event("native_telemetry_gap_recovered",
                       duration_s=now - self.native_gap_started)
            self.native_gap_started = None
        status_path = Path("/proc", str(self.vision_pid), "status")
        if (not status_path.exists()
                or any(line.startswith("State:") and line.split()[1] == "Z"
                       for line in status_path.read_text().splitlines())):
            raise RuntimeError("Vision process exited")

