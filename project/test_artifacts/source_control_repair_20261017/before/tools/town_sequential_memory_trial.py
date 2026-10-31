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
    matched_arrival_evidence,
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


from town_trial_harness import Harness


node = Harness(out, vision_pid)
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
    for component in ("mapper", "graphnav_planner", "goal_standoff", "triangulation", "vision"):
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
        "/spot1/object_search/query", "/spot1/visible_target_approach", "/spot1/object_mask", "/spot1/remembered_object_candidates",
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
