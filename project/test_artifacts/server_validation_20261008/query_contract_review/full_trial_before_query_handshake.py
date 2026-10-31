"""Fresh in-process Town test: find a green house, then revisit a passively seen station."""
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from beamngpy import BeamNGpy
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
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
)
from wildos_nav2_adapter.graph_path_bridge import xy, yaw
from wildos_nav2_adapter.test_progress import ManeuverProgress
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads


session = Path(os.environ.get(
    "WILDOS_TEST_SESSION_DIR", '/home/ly/town_memory_validation_20261008'))
out = session / (sys.argv[1] if len(sys.argv) > 1 else "green_house_station_arrival_20261005")
out.mkdir(exist_ok=True)
vision_pid = int((session / "vision.pid").read_text())
install_fail_closed_reads(PrefixedLengthSocket)
beamng = BeamNGpy("127.0.0.1", 25252, socket_timeout=5)
beamng.open(launch=False)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


class Harness(Node):
    def __init__(self):
        super().__init__("green_house_station_arrival_test")
        self.phase = "setup"
        self.query = "green residential house"
        self.query_started_ns = 0
        self.state = {}
        self.events = []
        self.masks = {}
        self.views = {}
        self.frames = deque(maxlen=30)
        self.latest_frame = None
        self.latest_frame_stamp = None
        self.latest_overlay = None
        self.latest_overlay_stamp = None
        self.latest_overlay_query = None
        self.video = cv2.VideoWriter(
            str(out / "front_camera.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 5, (960, 640))
        if not self.video.isOpened():
            raise RuntimeError("Failed to open front_camera.mp4 writer")
        self.log = (out / "events.jsonl").open("w", encoding="utf-8")
        self.native_log = (out / "native_samples.jsonl").open("w", encoding="utf-8")
        self.sensor_timing_log = (out / "sensor_timing.jsonl").open("w", encoding="utf-8")
        self.sensor_timing_records = 0
        self.started_at = time.monotonic()
        self.last_native_write = 0.0
        self.native_gap_started = None
        self.native_fault_reported = False
        self.actuator_bad_since = None
        self.last_status = None
        self.arrival_statuses = []
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
        item = {"elapsed": time.monotonic() - self.started_at, "phase": self.phase,
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
                     or data.get("carrier_model") != "ideal_point"
                     or data.get("minimum_turning_radius_m") != 0
                     or data.get("planner_id") != "GridBased")):
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
        self.confirmation = dict(data, received_at=time.monotonic())
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
        self.frames.append((stamp, frame.copy()))
        self.latest_frame = frame
        self.latest_frame_stamp = stamp

    def record_camera(self):
        if self.latest_frame is not None:
            frame = self.latest_frame
            if (self.latest_overlay is not None and self.latest_overlay_query == self.query
                    and self.latest_frame_stamp is not None
                    and self.latest_overlay_stamp is not None
                    and abs(self.latest_frame_stamp - self.latest_overlay_stamp) <= 0.5):
                frame = self.latest_overlay
            self.video.write(cv2.resize(frame, (960, 640)))

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
        self.masks[msg.query] = current
        if self.frames:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            _, frame = min(self.frames, key=lambda pair: abs(pair[0] - stamp))
            sizes = [dim.size for dim in msg.object_mask.layout.dim]
            mask = np.asarray(msg.object_mask.data, dtype=np.uint8).reshape(sizes)[0, 0] > 0
            overlay = frame.copy()
            overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)
            self.latest_overlay = overlay
            self.latest_overlay_stamp = stamp
            self.latest_overlay_query = msg.query

    def save_target_images(self, query):
        if self.latest_frame is not None:
            cv2.imwrite(str(out / f"{query.replace(' ', '_')}_arrival_rgb.png"), self.latest_frame)
        if (self.latest_overlay is not None and self.latest_overlay_query == query
                and self.latest_frame_stamp is not None
                and self.latest_overlay_stamp is not None
                and abs(self.latest_frame_stamp - self.latest_overlay_stamp) <= 0.5):
            cv2.imwrite(str(out / f"{query.replace(' ', '_')}_arrival_mask.png"), self.latest_overlay)

    def set_bool(self, remote, name, value):
        if remote == "/wildos_cmd_vel_to_control":
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
        results = future.result().results
        if not all(result.successful for result in results):
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
        self.set_bool("/wildos_graph_path_bridge", "follow_path", False)
        self.set_bool("/wildos_cmd_vel_to_control", "enable", False)
        return self.spin(3, lambda: self.state.get("native", {}).get("speed", 99) < 0.1)

    def change_query(self, query):
        self.query = query
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.waypoint = None
        self.confirmation = None
        self.masks.pop(query, None)
        self.latest_overlay = None
        self.latest_overlay_stamp = None
        self.latest_overlay_query = None
        self.state.pop("selection", None)
        self.state.pop("path", None)
        if query == "gas station":
            self.state.pop("first_memory_selection_at", None)
            self.state.pop("first_memory_path_at", None)
        self.event("query_sent")
        self.query_pub.publish(String(data=query))

    def arrival_evidence(self, query, minimum_mask_count):
        confirmation = self.confirmation
        mask = self.masks.get(query, {})
        now = time.monotonic()
        if (not confirmation or confirmation.get("query") != query
                or confirmation.get("search_confirmed") is not True
                or now - confirmation.get("received_at", 0.0) >= 5.0
                or mask.get("count", 0) < minimum_mask_count or mask.get("pixels", 0) <= 0
                or now - mask.get("at", 0.0) >= 5.0
                or not mask_matches_confirmation(query, mask.get("observation_stamp_ns"), confirmation)):
            return None
        for item in reversed(self.arrival_statuses):
            if (now - item["received_at"] < 5.0
                    and arrival_status_matches(item["status"], query, confirmation,
                                               STANDOFF_ARRIVAL_TOLERANCE_M)):
                return item
        return None

    def watch(self):
        now = time.monotonic()
        native_age = now - self.state.get("native_at", 0.0)
        if self.state.get("runtime_profile_error"):
            raise RuntimeError("Ideal-point runtime profile mismatch: " +
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
        status_path = Path("/proc", str(vision_pid), "status")
        if (not status_path.exists()
                or any(line.startswith("State:") and line.strip().endswith("Z")
                       for line in status_path.read_text().splitlines())):
            raise RuntimeError("Vision process exited")


node = Harness()
bag = None
report = {
    "passed": False,
    "scenario": "Town gas-station spawn; unseeded green-house query then gas-station query",
    "vision_pid": vision_pid,
    "map_reload_count": 0,
    "vehicle_teleports_during_sequence": 0,
    "station_preseeded": False,
    "arrival_tolerance_m": STANDOFF_ARRIVAL_TOLERANCE_M,
    "spawn_tolerance_m": 1.25,
    "speed_tuning_performed": False,
}
try:
    # Keep the already loaded map and BeamNG bridge. Refresh only the graph
    # bridge, mapper, planner and in-process vision memory for an independent run.
    beamng.control.pause()
    restart = subprocess.run([sys.executable, str(session / "restart_component.py"), "graph_bridge"],
                             text=True, capture_output=True, timeout=30)
    node.event("component_restart", component="graph_bridge", returncode=restart.returncode,
               stdout=restart.stdout[-500:], stderr=restart.stderr[-500:])
    if restart.returncode:
        raise RuntimeError("Failed to restart graph bridge with patched code")
    node.spin(0.5)
    node.set_bool("/wildos_graph_path_bridge", "follow_path", False)
    node.set_bool("/wildos_cmd_vel_to_control", "enable", False)

    collision_profile = json.loads(beamng.control.queue_lua_command("""
local veh=scenetree.findObject('ego')
local out={profileAvailable=true,physicsBypassed=false}
if extensions.wildosKinematic then
  local state=extensions.wildosKinematic.status()
  out.physicsBypassed=state.physicsBypassed
  out.fixedNodeCount=state.nodeCount
  out.carrierModel=state.carrierModel
  out.minimumTurningRadius=state.minimumTurningRadius
  out.rosYawToClusterSign=state.rosYawToClusterSign
  out.rotationConvention=state.rotationConvention
  out.visualMeshesHidden=state.visualMeshesHidden
  out.visualMeshCount=state.visualMeshCount
  out.hiddenMeshCount=state.hiddenMeshCount
end
out.mission=getMissionFilename()
out.collisionMasks={}
for _,node in pairs(core_vehicle_manager.getVehicleData(veh:getID()).vdata.nodes) do
  local mask=tostring(veh:getInitialNodeCollision(node.cid))
  out.collisionMasks[mask]=(out.collisionMasks[mask] or 0)+1
end
out.cameras=extensions.tech_sensors.getActiveCameraSensors()
out.lidars=extensions.tech_sensors.getActiveLidarSensors()
return jsonEncode(out)
""", response=True))
    report["collision_profile_before_sequence"] = collision_profile
    if not collision_profile.get("profileAvailable"):
        # Older BeamNG vehicle extensions do not expose wildosBodyCollision;
        # the live node mask counts below still record the actual state.
        collision_profile["profileNote"] = "wildosBodyCollision extension unavailable; using live node masks"
    if not collision_profile.get("cameras") or not collision_profile.get("lidars"):
        raise RuntimeError("Camera or LiDAR is unavailable")
    if (collision_profile.get("rosYawToClusterSign") != -1
            or collision_profile.get("rotationConvention") != "world-z-passive-v2"):
        raise RuntimeError("BeamNG has not loaded the corrected ROS yaw convention; reload the kinematic Lua extension before testing")
    if (collision_profile.get("carrierModel") != "ideal_point"
            or collision_profile.get("minimumTurningRadius") != 0
            or collision_profile.get("visualMeshesHidden") is not True):
        raise RuntimeError("BeamNG is not running the no-radius ideal sensor-carrier profile: "
                           + json.dumps({key: collision_profile.get(key) for key in
                                         ("carrierModel", "minimumTurningRadius",
                                          "visualMeshesHidden", "visualMeshCount", "hiddenMeshCount")}))
    if os.environ.get("WILDOS_REQUIRE_BODY_COLLISION_DISABLED") == "1":
        if (not collision_profile.get("physicsBypassed")
                or collision_profile["collisionMasks"].get("0", 0) != collision_profile.get("fixedNodeCount")):
            raise RuntimeError("All vehicle node physics is not bypassed")

    beamng.vehicles.teleport("ego", (734.294, -11.946, 52.154),
                              (0.0, 0.0, 0.7658087977066911, 0.6430683364581344), reset=True)
    beamng.control.resume()
    node.spin(2.0)
    for component in ("mapper", "graphnav_planner", "goal_standoff", "vision"):
        result = subprocess.run([sys.executable, str(session / "restart_component.py"), component],
                                text=True, capture_output=True, timeout=30)
        node.event("component_restart", component=component, returncode=result.returncode,
                   stdout=result.stdout[-500:], stderr=result.stderr[-500:])
        if result.returncode:
            raise RuntimeError("Failed to reset fresh trial component: " + component)
    vision_pid = int((session / "vision.pid").read_text())
    report["vision_pid"] = vision_pid
    report["fresh_in_process_memory"] = True
    node.clear_costmaps()
    node.spin(3.0)
    if not node.spin(12.0, lambda: bool(node.frames) and "native" in node.state
                     and "actuator_ok" in node.state["native"]):
        raise RuntimeError("No camera or vehicle state after same-map reset")

    native = node.state["native"]
    if (native.get("actuator_ok") is not True or native.get("telemetry_ok") is not True
            or native.get("control_error") is not None):
        raise RuntimeError("Native vehicle control contract is unhealthy before search: "
                           + str(native.get("actuator_error") or native.get("control_error")))
    if (native.get("carrier_model") != "ideal_point"
            or native.get("minimum_turning_radius_m") != 0
            or native.get("beamng_carrier_model") != "ideal_point"
            or native.get("beamng_minimum_turning_radius_m") != 0
            or native.get("beamng_visual_meshes_hidden") is not True):
        raise RuntimeError("Controller and BeamNG do not both report the ideal sensor-carrier profile: "
                           + json.dumps({key: native.get(key) for key in
                                         ("carrier_model", "minimum_turning_radius_m",
                                          "beamng_carrier_model", "beamng_minimum_turning_radius_m",
                                          "beamng_visual_meshes_hidden")}))
    report["runtime_profile"] = {
        "carrier_model": native.get("carrier_model"),
        "minimum_turning_radius_m": native.get("minimum_turning_radius_m"),
        "beamng_carrier_model": native.get("beamng_carrier_model"),
        "beamng_minimum_turning_radius_m": native.get("beamng_minimum_turning_radius_m"),
        "visual_meshes_hidden": native.get("beamng_visual_meshes_hidden"),
        "planner_id_expected": "GridBased",
    }
    born = native["position"][:2]
    report["start_position"] = born
    report["spawn_distance_m"] = math.dist(born, (734.294, -11.946))
    if report["spawn_distance_m"] > report["spawn_tolerance_m"]:
        raise RuntimeError("Vehicle is not at the requested Town gas-station spawn")

    # The triangulator also publishes a legacy PoseStamped beside its
    # query-scoped localization JSON. That pose currently has no consumer;
    # retain it in the bag only to compare the two triangulator outputs.
    # GoalStandoff consumes object_localization and publishes the usable route.
    topics = [
        "/spot1/object_search/query", "/spot1/object_mask", "/spot1/remembered_object_candidates",
        "/spot1/remembered_object_feedback", "/wildos/memory_selection", "/spot1/imgnav_waypoint",
        "/spot1/odom", "/spot1/local_map", "/spot1/graphnav_planner/path",
        "/spot1/graphnav_planner/route_status",
        "/spot1/graphnav_planner/frontier_scores",
        "/spot1/graphnav_planner/rejected_route_endpoint",
        "/spot1/nav_graph", "/spot1/within_range_geofrontiers",
        "/spot1/execution_anchor_graph",
        "/spot1/raw_object_waypoint", "/spot1/object_localization",
        "/spot1/object_confirmation", "/wildos/control_status",
        "/wildos/execution_feedback", "/wildos/plan_status", "/wildos/cmd_vel",
        "/wildos/native_control", "/wildos/native_control_status", "/global_costmap/costmap",
        "/local_costmap/costmap", "/tf", "/tf_static", "/vehicles/ego/sensors/state",
        "/spot1/measurement_timing", "/spot1/local_terrain", "/wildos/kinematic_state",
        "/wildos/planned_path", "/wildos/full_planned_path",
        "/spot1/graphnav_destination", "/spot1/remembered_object_goal",
        "/global_costmap/published_footprint", "/local_costmap/published_footprint",
        "/plan", "/received_global_plan", "/lookahead_point",
        "/lookahead_collision_arc", "/speed_limit", "/parameter_events", "/rosout",
        "/spot1/realsense/front/color/camera_info",
        "/spot1/ouster/front/points_filtered",
        "/compute_path_to_pose/_action/status", "/compute_path_to_pose/_action/feedback",
        "/follow_path/_action/status", "/follow_path/_action/feedback",
    ]
    report["topics_recorded"] = topics
    with (out / "rosbag.log").open("ab") as bag_log:
        bag = subprocess.Popen(["ros2", "bag", "record", "--include-hidden-topics",
                                "-o", str(out / "rosbag"), *topics],
                              stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
    if not node.spin(12.0, lambda: any(
            "rosbag2_recorder" in info.node_name
            for info in node.get_subscriptions_info_by_topic("/spot1/object_search/query"))):
        raise RuntimeError("ROS bag recorder did not subscribe to the test query")

    node.phase = "green_house_search"
    node.change_query("green residential house")
    report["queries_sent"] = ["green residential house"]
    if not node.spin(6.0, lambda: bool(node.last_status)
                     and node.last_status.get("query") == "green residential house"):
        raise RuntimeError("Fresh bridge did not accept the green-house query")
    status = node.last_status
    if (status.get("schema_version") != 2
            or status.get("carrier_model") != "ideal_point"
            or status.get("minimum_turning_radius_m") != 0
            or status.get("planner_id") != "GridBased"):
        raise RuntimeError("GraphPathBridge runtime profile is stale or incompatible: " +
                           json.dumps({key: status.get(key) for key in
                                       ("schema_version", "carrier_model",
                                        "minimum_turning_radius_m", "planner_id")}))
    # Keep the actuator and FollowPath disabled until the running bridge has
    # proved its model and planner schema for this query.
    node.set_bool("/wildos_cmd_vel_to_control", "enable", True)
    node.set_bool("/wildos_graph_path_bridge", "follow_path", True)
    search_started = time.monotonic()
    last_progress = search_started
    best_distance = 0.0
    maximum_departure = 0.0
    last_report = 0.0
    last_progress_status = None
    maneuver_progress = ManeuverProgress()
    green_house_arrival = None
    deadline = search_started + 420.0
    while time.monotonic() < deadline:
        node.spin(0.1)
        node.watch()
        status = node.last_status or {}
        status_marker = (status.get("status"), tuple(status.get("goal") or ()),
                         status.get("view_id"))
        if status_marker != last_progress_status:
            last_progress_status = status_marker
            if status.get("status") in (
                    "awaiting_visual_confirmation", "memory_observation_approach",
                    "memory_observation_exhausted", "unconfirmed_target_resume_exploration",
                    "recovery_started", "recovery_done"):
                # Arrival at an observation pose and a new checked recovery
                # attempt are meaningful search progress even while stopped.
                last_progress = time.monotonic()
        position = node.state["native"].get("position")
        if not isinstance(position, (list, tuple)) or len(position) < 2:
            node.event("invalid_native_position", native=node.state["native"])
            continue
        point = position[:2]
        departure = math.dist(point, born)
        maximum_departure = max(maximum_departure, departure)
        if departure > best_distance + 0.3:
            best_distance = departure
            last_progress = time.monotonic()
        if "odom" in node.state and maneuver_progress.observe(xy(node.state["odom"].pose.pose), node.last_status):
            last_progress = time.monotonic()
        if time.monotonic() - last_report > 10.0:
            node.event("search_progress", position=point, departure_m=departure,
                       speed=node.state["native"].get("speed"), status=node.last_status,
                       mask_count=node.masks.get("green residential house", {}).get("count", 0))
            last_report = time.monotonic()
        green_house_arrival = node.arrival_evidence(
            "green residential house", minimum_mask_count=3)
        if green_house_arrival:
            node.save_target_images("green residential house")
            report["green_house_arrival"] = {
                "status": green_house_arrival["status"],
                "confirmation": node.confirmation,
                "mask": node.masks.get("green residential house"),
                "vehicle_position": node.state["native"]["position"],
            }
            break
        if time.monotonic() - last_progress > 65.0:
            node.event("search_no_progress", status=node.last_status)
            break
    node.stop()
    green_house_found = green_house_arrival is not None
    report.update(green_house_found=green_house_found, maximum_departure_m=maximum_departure,
                  green_house_search_elapsed_s=time.monotonic() - search_started,
                  green_house_end_position=node.state.get("native", {}).get("position"))
    node.event("green_house_search_finished", found=green_house_found,
               maximum_departure_m=maximum_departure)

    # A second goal is only issued after the first goal passed the strict
    # same-observation mask, confirmation, object_found and standoff checks.
    if green_house_found:
        node.set_bool("/wildos_cmd_vel_to_control", "enable", True)
        node.set_bool("/wildos_graph_path_bridge", "follow_path", True)
        node.phase = "station_return"
        node.change_query("gas station")
        report["queries_sent"].append("gas station")
        switched_at = time.monotonic()
        if not node.spin(6.0, lambda: bool(node.last_status)
                         and node.last_status.get("query") == "gas station"):
            raise RuntimeError("Fresh bridge did not accept the gas-station query")
        last_progress = switched_at
        best_distance = math.dist(node.state["native"]["position"][:2], born)
        last_report = 0.0
        last_progress_status = None
        maneuver_progress = ManeuverProgress()
        station_arrival = None
        deadline = time.monotonic() + 420.0
        while time.monotonic() < deadline:
            node.spin(0.1)
            node.watch()
            status = node.last_status or {}
            status_marker = (status.get("status"), tuple(status.get("goal") or ()),
                             status.get("view_id"))
            if status_marker != last_progress_status:
                last_progress_status = status_marker
                if status.get("status") in (
                        "awaiting_visual_confirmation", "memory_observation_approach",
                        "memory_observation_exhausted", "unconfirmed_target_resume_exploration",
                        "recovery_started", "recovery_done"):
                    last_progress = time.monotonic()
            position = node.state["native"].get("position")
            if not isinstance(position, (list, tuple)) or len(position) < 2:
                node.event("invalid_native_position", native=node.state["native"])
                continue
            point = position[:2]
            return_distance = math.dist(point, born)
            if return_distance < best_distance - 0.3:
                best_distance = return_distance
                last_progress = time.monotonic()
            if "odom" in node.state and maneuver_progress.observe(xy(node.state["odom"].pose.pose), node.last_status):
                last_progress = time.monotonic()
            if time.monotonic() - last_report > 10.0:
                node.event("return_progress", position=point, speed=node.state["native"].get("speed"),
                           status=node.last_status, selection=node.state.get("selection"))
                last_report = time.monotonic()
            station_arrival = node.arrival_evidence("gas station", minimum_mask_count=2)
            if station_arrival:
                status = station_arrival["status"]
                selection = node.state.get("selection", {})
                selected_id = selection.get("view_id", "")
                status_view_id = status.get("memory_view_id", "")
                selected_parent = selected_id.split("~approach", 1)[0]
                status_parent = status_view_id.split("~approach", 1)[0]
                memory_delta = status.get("memory_region_distance_m")
                memory_tolerance = status.get("memory_region_tolerance_m")
                region_match = (selection.get("query") == "gas station"
                                and selection.get("status") == "memory_candidate_selected"
                                and bool(selected_id) and status_parent == selected_parent
                                and isinstance(memory_delta, (int, float))
                                and isinstance(memory_tolerance, (int, float))
                                and math.isfinite(memory_delta) and math.isfinite(memory_tolerance)
                                and memory_delta <= memory_tolerance)
                if region_match:
                    node.save_target_images("gas_station")
                    report["station_arrival"] = {
                        "status": status,
                        "confirmation": node.confirmation,
                        "mask": node.masks.get("gas station"),
                        "selection": selection,
                        "memory_view_id_match": True,
                        "memory_region_distance_m": memory_delta,
                        "memory_region_tolerance_m": memory_tolerance,
                        "vehicle_position": node.state["native"]["position"],
                    }
                    break
                node.event("station_arrival_rejected", memory_view_id=status_view_id,
                           selected_view_id=selected_id, memory_region_distance_m=memory_delta,
                           memory_region_tolerance_m=memory_tolerance)
            if time.monotonic() - last_progress > 65.0:
                node.event("return_no_progress", status=node.last_status)
                break
        node.stop()
        station_found = bool(report.get("station_arrival"))
        selected_at = node.state.get("first_memory_selection_at")
        path_at = node.state.get("first_memory_path_at")
        report.update(
            gas_station_found=station_found,
            return_elapsed_s=time.monotonic() - switched_at,
            memory_selection_latency_s=(selected_at - switched_at if selected_at else None),
            memory_path_latency_s=(path_at - switched_at if path_at else None),
            memory_navigation_exercised=path_at is not None,
            station_candidates_at_switch=node.views.get("gas station"),
        )
    else:
        report.update(gas_station_found=False, second_phase_skipped=True,
                      failure_reason="green_house_did_not_pass_strict_arrival_check")

    report["vision_process_retained"] = Path("/proc", str(vision_pid)).exists()
    report["final_position"] = node.state.get("native", {}).get("position")
    report["passed"] = bool(
        report.get("green_house_found") and report.get("gas_station_found")
        and report.get("memory_navigation_exercised")
        and report.get("vision_process_retained")
        and report.get("map_reload_count") == 0
        and report.get("vehicle_teleports_during_sequence") == 0)
    node.event("full_test_finished", **{k: v for k, v in report.items() if k != "topics_recorded"})
except BaseException as exc:
    report["error"] = repr(exc)
    try:
        node.event("test_error", error=repr(exc))
    except Exception:
        pass
finally:
    try:
        node.stop()
    except Exception as exc:
        report["stop_error"] = repr(exc)
    try:
        beamng.control.pause()
        report["simulation_paused"] = True
    except Exception as exc:
        report["pause_error"] = repr(exc)
    try:
        beamng.disconnect()
    except Exception:
        pass
    if bag is not None:
        try:
            bag.send_signal(signal.SIGINT)
            bag.wait(timeout=15)
        except subprocess.TimeoutExpired:
            bag.terminate()
        except ProcessLookupError:
            pass
    report["events"] = node.events
    report["queries_sent"] = [item["query"] for item in node.events if item["kind"] == "query_sent"]
    report["vision_process_retained"] = Path("/proc", str(vision_pid)).exists()
    (out / "full_test.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    node.video.release()
    node.log.close()
    node.native_log.close()
    node.sensor_timing_log.flush()
    node.sensor_timing_log.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

print(json.dumps({k: v for k, v in report.items() if k != "events"}, indent=2), flush=True)
raise SystemExit(0 if report.get("passed") else 1)
