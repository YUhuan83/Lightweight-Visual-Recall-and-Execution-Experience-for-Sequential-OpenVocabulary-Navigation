"""Capture RViz while running the strict same-map green-house-to-station test."""
from pathlib import Path
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path("/home/ly/town_memory_validation_20261003")
WS = Path("/home/ly/beamng_ros2_0394_ws")
name = sys.argv[1] if len(sys.argv) > 1 else "green_house_station_arrival_20261006_kinematic1"
if not name.replace("_", "").isalnum():
    raise ValueError("Invalid test name")
out = ROOT / name
if out.exists():
    raise RuntimeError("Test output already exists: " + str(out))
out.mkdir()
runtime_env = json.loads(Path("/home/ly/full_town_memory_20261002/runtime_env.json").read_text())
env = dict(os.environ, **runtime_env)
require_body_collision_disabled = os.environ.get(
    "WILDOS_REQUIRE_BODY_COLLISION_DISABLED", "0")
env.update(ROS_DOMAIN_ID="40", ROS_LOCALHOST_ONLY="1", ROS2CLI_NO_DAEMON="1",
           WILDOS_TEST_SESSION_DIR=str(ROOT),
           WILDOS_REQUIRE_BODY_COLLISION_DISABLED=require_body_collision_disabled)


def ros(*args, timeout=6):
    result = subprocess.run(["ros2", *args], cwd=WS, env=env, text=True,
                            capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(f"ROS preflight failed: ros2 {' '.join(args)}: "
                           f"{result.stdout[-500:]} {result.stderr[-500:]}")
    return result.stdout.strip()


def require_param(node, name, expected):
    actual = ros("param", "get", node, name)
    if expected not in actual:
        raise RuntimeError(f"Runtime contract mismatch: {node}.{name}: "
                           f"expected {expected!r}, got {actual!r}")
    return actual


def enforce_param(node, name, value, expected):
    previous = ros("param", "get", node, name)
    changed = expected not in previous
    if changed:
        ros("param", "set", node, name, value)
    actual = require_param(node, name, expected)
    return {"node": node, "parameter": name, "before": previous,
            "after": actual, "changed": changed}


def keep_disabled(node, parameter):
    actual = ros("param", "get", node, parameter)
    if "Boolean value is: True" in actual:
        ros("param", "set", node, parameter, "false")
    return require_param(node, parameter, "False")


def require_publisher(topic):
    info = ros("topic", "info", topic)
    if "Publisher count: 1" not in info:
        raise RuntimeError(f"Expected one publisher on {topic}, got: {info}")
    return info


def require_counts(topic, min_publishers, min_subscribers):
    info = ros("topic", "info", topic)
    pub_count = int(info.split("Publisher count:", 1)[1].splitlines()[0].strip())
    sub_count = int(info.split("Subscription count:", 1)[1].splitlines()[0].strip())
    if pub_count < min_publishers or sub_count < min_subscribers:
        raise RuntimeError(f"ROS topic chain incomplete for {topic}: {info}")
    return info


owned_kinematic_pid = None
preflight = {"map_reload_count": 0, "vehicle_motion_enabled": False}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def capture_runtime_parameters(label, node):
    try:
        snapshot = ros("param", "dump", node, timeout=12)
        directory = out / "runtime_parameters"
        directory.mkdir(exist_ok=True)
        (directory / f"{label}.yaml").write_text(snapshot + "\n", encoding="utf-8")
        return {"node": node, "captured": True}
    except (RuntimeError, subprocess.TimeoutExpired) as exc:
        return {"node": node, "captured": False, "error": repr(exc)}


try:
    nodes = ros("node", "list").splitlines()
    keep_disabled("/wildos_graph_path_bridge", "follow_path")
    require_param("/wildos_graph_path_bridge", "carrier_model", "ideal_point")
    require_param("/wildos_graph_path_bridge", "minimum_turning_radius", "0.0")
    require_param("/planner_server", "planner_plugins", "GridBased")
    planner_plugins = ros("param", "get", "/planner_server", "planner_plugins")
    if "CarLike" in planner_plugins:
        raise RuntimeError("CarLike is still loaded in the ideal-point planner profile")
    controller_contract = [
        enforce_param("/controller_server", "FollowPath.allow_reversing",
                      "false", "Boolean value is: False"),
        enforce_param("/controller_server", "FollowPath.use_rotate_to_heading",
                      "true", "Boolean value is: True"),
        enforce_param("/controller_server", "FollowPath.use_collision_detection",
                      "false", "Boolean value is: False"),
        enforce_param("/controller_server", "FollowPath.desired_linear_vel",
                      "1.2", "Double value is: 1.2"),
    ]

    if "/wildos_kinematic_vehicle_control" not in nodes:
        kinematic_log = (out / "kinematic.log").open("w")
        process = subprocess.Popen([
            str(WS / ".venv/bin/python"), "-m",
            "wildos_nav2_adapter.kinematic_vehicle_control",
            "--ros-args", "-p", "enable:=false"], cwd=WS, env=env,
            stdout=kinematic_log, stderr=subprocess.STDOUT, start_new_session=True)
        owned_kinematic_pid = process.pid
        (out / "kinematic.pid").write_text(str(process.pid))
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Kinematic controller exited during startup: "
                                   + (out / "kinematic.log").read_text()[-1500:])
            try:
                nodes = ros("node", "list", timeout=3).splitlines()
                if "/wildos_kinematic_vehicle_control" in nodes:
                    break
            except (RuntimeError, subprocess.TimeoutExpired):
                pass
            time.sleep(0.5)
        else:
            raise RuntimeError("Kinematic controller did not appear in the ROS graph")
        kinematic_log.close()

    require_param("/wildos_kinematic_vehicle_control", "carrier_model", "ideal_point")
    require_param("/wildos_kinematic_vehicle_control", "minimum_turning_radius", "0.0")
    keep_disabled("/wildos_kinematic_vehicle_control", "enable")
    require_counts("/vehicles/ego/sensors/state", 1, 1)
    require_counts("/wildos/kinematic_state", 1, 1)
    require_publisher("/vehicles/ego/sensors/front_cam/colour")
    require_publisher("/vehicles/ego/sensors/front_lidar")
    preflight.update({"passed": True, "nodes": nodes,
                      "carrier_model": "ideal_point", "planner_plugins": planner_plugins,
                      "minimum_turning_radius_m": 0.0,
                      "kinematic_controller_started_by_runner": owned_kinematic_pid is not None,
                      "controller_contract": controller_contract})
    provenance = {
        "runner_sha256": sha256(__file__),
        "harness_sha256": sha256(ROOT / "green_house_station_arrival_kinematic_test.py"),
    }
    source_bridge = WS / "src/wildos_nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py"
    installed_control = WS / "install/wildos_nav2_adapter/share/wildos_nav2_adapter/config/nav2_official_control.yaml"
    for label, path in (("graph_path_bridge_source", source_bridge),
                        ("nav2_official_control_installed", installed_control)):
        if path.is_file():
            provenance[label + "_sha256"] = sha256(path)
    preflight["provenance"] = provenance
    preflight["runtime_parameter_snapshots"] = [
        capture_runtime_parameters(label, node)
        for label, node in (
            ("planner_server", "/planner_server"),
            ("controller_server", "/controller_server"),
            ("graph_path_bridge", "/wildos_graph_path_bridge"),
            ("kinematic_vehicle_control", "/wildos_kinematic_vehicle_control"),
            ("cmd_vel_to_control", "/wildos_cmd_vel_to_control"),
            ("global_costmap", "/global_costmap/global_costmap"),
            ("local_costmap", "/local_costmap/local_costmap"),
            ("graphnav_planner", "/spot1/graphnav_planner"),
        ) if node in nodes
    ]
except BaseException as exc:
    preflight.update(passed=False, error=repr(exc))
    (out / "preflight.json").write_text(json.dumps(preflight, indent=2))
    if owned_kinematic_pid is not None:
        try:
            os.kill(owned_kinematic_pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    raise
(out / "preflight.json").write_text(json.dumps(preflight, indent=2))

with (out / "rviz_recording.log").open("w") as recorder_log:
    recorder = subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "warning", "-f", "x11grab",
        "-filter_threads", "1", "-video_size", "1280x1024", "-framerate", "5",
        "-i", ":99", "-t", "1100", "-c:v", "libx264", "-threads", "1",
        "-preset", "ultrafast", "-pix_fmt", "yuv420p", str(out / "rviz_live.mp4")],
        stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
(out / "recorder.pid").write_text(str(recorder.pid))
time.sleep(0.7)
if recorder.poll() is not None:
    if owned_kinematic_pid is not None:
        try:
            os.kill(owned_kinematic_pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    raise RuntimeError((out / "rviz_recording.log").read_text())

trial = None
try:
    with (out / "trial.log").open("w") as trial_log:
        trial = subprocess.Popen([
            str(WS / ".venv/bin/python"),
            str(ROOT / "green_house_station_arrival_kinematic_test.py"), name],
            cwd=WS, env=env, stdout=trial_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        (out / "trial.pid").write_text(str(trial.pid))
        print(json.dumps({"name": name, "trial_pid": trial.pid,
                          "recorder_pid": recorder.pid}), flush=True)
        try:
            result = trial.wait(timeout=950)
        except subprocess.TimeoutExpired as exc:
            trial.terminate()
            try:
                trial.wait(timeout=10)
            except subprocess.TimeoutExpired:
                trial.kill()
                trial.wait(timeout=5)
            raise RuntimeError("Town trial exceeded its 950 second limit") from exc
    print(json.dumps({"trial_exit": result}), flush=True)
    raise SystemExit(result)
finally:
    if trial is not None and trial.poll() is None:
        trial.send_signal(signal.SIGINT)
        try:
            trial.wait(timeout=20)
        except subprocess.TimeoutExpired:
            trial.terminate()
            try:
                trial.wait(timeout=5)
            except subprocess.TimeoutExpired:
                trial.kill()
                trial.wait(timeout=5)
    if recorder.poll() is None:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=15)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            recorder.wait(timeout=5)
    if owned_kinematic_pid is not None:
        try:
            os.kill(owned_kinematic_pid, signal.SIGINT)
        except ProcessLookupError:
            pass
