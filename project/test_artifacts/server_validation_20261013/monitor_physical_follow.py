"""Safely arm and monitor one already planned physical FollowPath segment."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String


def bool_parameter(name, value):
    return Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=value))


def parse_json(message):
    try:
        return json.loads(message.data)
    except (ValueError, TypeError):
        return {"raw": message.data[:500]}


def native_speed(status):
    return float(status.get("speed_m_s", status.get("speed", math.inf)))


def native_brake(status):
    value = status.get("actual_brake")
    if value is None:
        value = status.get("actual", {}).get("brake", 0.0)
    return float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)

    rclpy.init()
    node = rclpy.create_node("bounded_physical_follow_monitor")
    latest = {"odom": None, "plan": None, "native": None, "control": None, "actions": []}
    received = {"odom": 0.0, "plan": 0.0, "native": 0.0, "control": 0.0}
    events = []
    action_states = []

    def odom_cb(msg):
        p = msg.pose.pose.position
        latest["odom"] = {"stamp_ns": msg.header.stamp.sec * 1_000_000_000
                           + msg.header.stamp.nanosec,
                           "frame_id": msg.header.frame_id,
                           "position": [float(p.x), float(p.y), float(p.z)]}
        received["odom"] = time.monotonic()

    def json_cb(key):
        def callback(msg):
            latest[key] = parse_json(msg)
            received[key] = time.monotonic()
            if key == "plan":
                item = {"elapsed_s": round(time.monotonic() - run_started, 3),
                        "status": latest[key].get("status"),
                        "goal": latest[key].get("goal"),
                        "detail": latest[key].get("detail")}
                if not events or events[-1] != item:
                    events.append(item)
        return callback

    def action_cb(msg):
        states = [int(item.status) for item in msg.status_list]
        latest["actions"] = states
        if states and (not action_states or action_states[-1]["states"] != states):
            action_states.append({"elapsed_s": round(time.monotonic() - run_started, 3),
                                  "states": states})

    run_started = time.monotonic()
    node.create_subscription(Odometry, "/spot1/odom", odom_cb, 20)
    node.create_subscription(String, "/wildos/plan_status", json_cb("plan"), 20)
    node.create_subscription(String, "/wildos/native_control_status", json_cb("native"), 20)
    node.create_subscription(String, "/wildos/control_status", json_cb("control"), 20)
    node.create_subscription(GoalStatusArray, "/follow_path/_action/status", action_cb, 10)

    bridge_get = node.create_client(GetParameters, "/wildos_graph_path_bridge/get_parameters")
    control_get = node.create_client(GetParameters, "/wildos_cmd_vel_to_control/get_parameters")
    bridge_set = node.create_client(SetParameters, "/wildos_graph_path_bridge/set_parameters")
    control_set = node.create_client(SetParameters, "/wildos_cmd_vel_to_control/set_parameters")

    report = {"passed": False, "armed": False, "visual_arrival_confirmed": False,
              "planner_id": "CarLike", "carrier_model": "ackermann",
              "minimum_turning_radius_m": 8.0, "events": events,
              "action_states": action_states, "samples": []}

    def call(client, request, label, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(label + " service unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(label + " timed out")
        response = future.result()
        if hasattr(response, "results") and any(not item.successful for item in response.results):
            reasons = [item.reason for item in response.results if not item.successful]
            raise RuntimeError(label + " rejected: " + "; ".join(reasons))
        return response

    def set_bool(client, name, value):
        call(client, SetParameters.Request(parameters=[bool_parameter(name, value)]),
             name + "=" + str(value))

    def disable_gates():
        errors = []
        try:
            set_bool(control_set, "enable", False)
        except Exception as exc:  # Retain the first stop attempt's evidence.
            errors.append("control disable: " + str(exc))
        try:
            set_bool(bridge_set, "follow_path", False)
        except Exception as exc:
            errors.append("FollowPath disable: " + str(exc))
        return errors

    try:
        warm_deadline = time.monotonic() + 5.0
        while time.monotonic() < warm_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (latest["odom"] and latest["plan"] and latest["native"]
                    and latest["control"] and received["odom"] > 0
                    and received["plan"] > 0 and received["native"] > 0):
                break
        for key in ("odom", "plan", "native", "control"):
            if latest[key] is None or time.monotonic() - received[key] > 1.0:
                raise RuntimeError("stale or missing preflight input: " + key)

        bridge_params = call(bridge_get, GetParameters.Request(names=["follow_path"]),
                             "read FollowPath gate").values
        control_params = call(control_get, GetParameters.Request(names=["enable"]),
                              "read actuator gate").values
        if bridge_params[0].bool_value or control_params[0].bool_value:
            raise RuntimeError("preflight requires both execution gates closed")
        plan = latest["plan"]
        if (plan.get("status") != "planned" or plan.get("carrier_model") != "ackermann"
                or plan.get("minimum_turning_radius_m") != 8.0
                or plan.get("planner_id") != "CarLike" or int(plan.get("poses", 0)) < 2):
            raise RuntimeError("no current, geometry-matched planned route: " + json.dumps(plan))
        native = latest["native"]
        if (native.get("actuator_ok") is not True
                or native.get("control_acknowledged") is not True
                or native.get("telemetry_ok") is not True
                or native_speed(native) > 0.03
                or native_brake(native) < 0.9):
            raise RuntimeError("vehicle is not healthy and held before motion: " + json.dumps(native))
        start = list(latest["odom"]["position"][:2])
        target = list(plan["goal"])
        initial_distance = math.dist(start, target)
        if not 8.0 <= initial_distance <= 20.0:
            raise RuntimeError("planned target outside bounded test range: %.3f m" % initial_distance)
        report.update({"start_xy": start, "target_xy": target,
                       "initial_distance_m": initial_distance,
                       "planned_pose_count": int(plan["poses"]),
                       "preflight_native": native,
                       "preflight_plan": plan})

        # Keep the actuator gated until its fresh controller goal is ready.
        set_bool(control_set, "enable", True)
        set_bool(bridge_set, "follow_path", True)
        report["armed"] = True
        motion_start = None
        last_sample = 0.0
        arrival_at = None
        steady_stop_since = None
        failure_statuses = {"controller_unavailable", "no_vehicle_path", "no_free_graph_candidate",
                            "path_changed_before_execution", "execution_unavailable",
                            "stale_odom", "stale_costmap", "stale_local_costmap"}
        overall_deadline = time.monotonic() + 45.0
        while time.monotonic() < overall_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            plan = latest["plan"] or {}
            native = latest["native"] or {}
            control = latest["control"] or {}
            odom = latest["odom"]
            if (not odom or now - received["odom"] > 0.75
                    or now - received["native"] > 0.75
                    or now - received["control"] > 0.75):
                report["failure"] = "execution telemetry stale"
                break
            distance_travelled = math.dist(start, odom["position"][:2])
            distance_to_goal = math.dist(target, odom["position"][:2])
            if (now - last_sample) >= 0.2:
                report["samples"].append({
                    "elapsed_s": round(now - run_started, 3),
                    "position": odom["position"],
                    "distance_travelled_m": distance_travelled,
                    "distance_to_goal_m": distance_to_goal,
                    "plan_status": plan.get("status"),
                    "native_speed_m_s": native.get("speed_m_s"),
                    "control_enabled": control.get("enabled"),
                    "control_actuating": control.get("actuating"),
                    "control_target_speed_m_s": control.get("target_speed"),
                    "control_brake": control.get("brake"),
                })
                last_sample = now
            if plan.get("status") == "following" and motion_start is None:
                motion_start = now
            if plan.get("status") in failure_statuses:
                report["failure"] = "Bridge status: " + str(plan.get("status"))
                break
            if distance_travelled > 20.0:
                report["failure"] = "bounded displacement exceeded 20 m"
                break
            if motion_start is not None and control.get("actuating") is True:
                if distance_travelled < 0.25 and now - motion_start > 6.0:
                    report["failure"] = "actuation active without 0.25 m progress for 6 s"
                    break
            if plan.get("status") in ("route_done", "awaiting_visual_confirmation") and distance_to_goal <= 1.0:
                if arrival_at is None:
                    arrival_at = now
                    report["geometric_arrival_observed"] = True
                stopped = native_speed(native) <= 0.05 and native_brake(native) >= 0.9
                if stopped:
                    steady_stop_since = steady_stop_since or now
                else:
                    steady_stop_since = None
                if steady_stop_since is not None and now - steady_stop_since >= 1.0:
                    report["stopped_at_goal_observed"] = True
                    break
            if motion_start is None and now - run_started > 10.0:
                report["failure"] = "FollowPath did not enter following state within 10 s"
                break
        else:
            report["failure"] = "45 s test deadline exceeded"

        report["follow_succeeded"] = any(4 in item["states"] for item in action_states)
        report["action_states"] = action_states
        report["events"] = events
        if latest["odom"]:
            report["final_xy"] = latest["odom"]["position"][:2]
            report["final_distance_to_goal_m"] = math.dist(target, latest["odom"]["position"][:2])
            report["travelled_m"] = math.dist(start, latest["odom"]["position"][:2])
        stop_errors = disable_gates()
        report["stop_errors"] = stop_errors
        settle_deadline = time.monotonic() + 2.0
        stop_ok_since = None
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            native = latest["native"] or {}
            if native_speed(native) <= 0.05 and native_brake(native) >= 0.9:
                stop_ok_since = stop_ok_since or time.monotonic()
            else:
                stop_ok_since = None
        report["final_native"] = latest["native"]
        report["motion_test_passed"] = bool(
            report.get("geometric_arrival_observed")
            and report.get("stopped_at_goal_observed")
            and report.get("follow_succeeded")
            and report.get("final_distance_to_goal_m", math.inf) <= 1.0
            and report.get("travelled_m", 0.0) >= 8.0
            and not stop_errors
            and stop_ok_since is not None)
        report["passed"] = report["motion_test_passed"]
    except Exception as exc:
        report["failure"] = str(exc)
        report["stop_errors"] = disable_gates()
    finally:
        try:
            report["final_plan"] = latest["plan"]
            report["final_control"] = latest["control"]
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
            print(json.dumps(report, indent=2))
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
