"""Run offline navigation checks; never imports ROS or drives a vehicle.
Usage: python tools/offline_navigation_check.py
Requires locally installed numpy and PyYAML. No downloads are performed.
"""
from __future__ import annotations
import ast
import hashlib
import importlib.util
import io
import json
import math
import re
import subprocess
import traceback
from pathlib import Path
import sys
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "test_artifacts" / "offline_navigation"
VIS = ROOT / "wildos_port/repro_check/src/nebula2-wildos/visual_navigation"
sys.path[:0] = [str(ROOT / "nav2_adapter"), str(VIS)]

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

def literal_parameter_defaults(tree, constants):
    """Read defaults without executing a ROS module; unknown names fail closed."""
    result = {}
    for node in ast.walk(tree):
        if (not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute)
                or node.func.attr != 'declare_parameter' or len(node.args) < 2):
            continue
        value = node.args[1]
        if isinstance(value, ast.Name):
            if value.id not in constants:
                raise ValueError('Unresolved parameter default: ' + value.id)
            value = constants[value.id]
        else:
            value = ast.literal_eval(value)
        result[ast.literal_eval(node.args[0])] = value
    return result

def main():
    import numpy as np
    import yaml
    from wildos_nav2_adapter.pointcloud import split_costmap_points
    OUT.mkdir(parents=True, exist_ok=True)
    results = []
    def check(name, fn):
        try:
            detail = fn()
            results.append(dict(name=name, status="PASS", detail=detail))
        except Exception as exc:
            results.append(dict(name=name, status="FAIL",
                                detail=f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}"))

    def warning(name, detail):
        results.append(dict(name=name, status="WARN", detail=detail))

    tests = [ROOT / "official_adapter/test/test_official_adapter.py",
             ROOT / "official_adapter/test/test_terrain.py",
             ROOT / "tools/test_beamng_kinematic_math.py",
             ROOT / "tools/test_offline_interface_replay.py",
             ROOT / "tools/test_physical_trial_policy.py",
              ROOT / "tools/test_wildos_console.py",
             ROOT / "tools/test_source_control_preflight.py",
             ROOT / "tools/test_offline_parameter_defaults.py",
             ROOT / "tools/test_package_official_bundle.py"]
    tests += [ROOT / "nav2_adapter/test" / (name + ".py") for name in
              ("test_adapters", "test_planning", "test_pointcloud",
               "test_graph_route", "test_memory_candidates", "test_cmd_vel_to_control",
               "test_motion_path", "test_test_progress", "test_sensor_lifecycle", "test_execution_recovery",
               "test_kinematic_rpc_worker", "test_kinematic_profile_status",
               "test_arrival_criteria", "test_source_control", "test_runtime_contract", "test_sensor_attachment_contract", "test_dds_profile",
               "test_scene_display", "test_rviz_configuration", "test_tracking_preview", "test_lidar_ray_origin")]
    tests += [VIS / "test" / (name + ".py") for name in
              ("test_visual_memory", "test_visual_memory_map", "test_remembered_goal", "test_viewpoint_memory",
               "test_relation_query", "test_frontier_history", "test_feedback_matching", "test_scoring_execution_contract",
               "test_directional_coverage_contract", "test_execution_prior_contract",
               "test_graph_snapshot_sync", "test_object_memory_integration", "test_localization_geometry",
               "test_semantic_landmark_memory", "test_target_bearing", "test_recall_component_names")]
    for i, path in enumerate(tests):
        def run_tests(path=path, i=i):
            module = load(path, f"offline_suite_{i}")
            stream = io.StringIO()
            result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
                unittest.defaultTestLoader.loadTestsFromModule(module))
            (OUT / (path.stem + ".log")).write_text(stream.getvalue(), encoding="utf8")
            if not result.wasSuccessful():
                raise AssertionError(f"{len(result.failures)} failures, {len(result.errors)} errors; see log")
            return dict(tests=result.testsRun, skipped=len(result.skipped))
        check(path.stem, run_tests)

    def message_filter_sync_test():
        path = VIS / "test" / "test_mask_lidar_delay.py"
        module = load(path, "offline_suite_mask_lidar_delay")
        stream = io.StringIO()
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromModule(module))
        (OUT / (path.stem + ".log")).write_text(stream.getvalue(), encoding="utf8")
        if not result.wasSuccessful():
            raise AssertionError(f"{len(result.failures)} failures, {len(result.errors)} errors; see log")
        return dict(tests=result.testsRun, skipped=len(result.skipped))
    if importlib.util.find_spec("message_filters") is None:
        warning("message_filter_mask_lidar_sync",
                "test_mask_lidar_delay.py needs ROS message_filters, absent in this Windows environment; run it with the Linux ROS 2 suite.")
    else:
        check("message_filter_mask_lidar_sync", message_filter_sync_test)

    def callback_contracts():
        # Keep interface doubles out of the main checker's import environment.
        result = subprocess.run([sys.executable, str(ROOT / 'tools/offline_callback_check.py')],
                                cwd=ROOT, capture_output=True, encoding='utf8', errors='replace', timeout=30)
        if result.returncode != 0:
            raise AssertionError('Callback tests failed: ' + (result.stdout + result.stderr)[-3000:])
        report = json.loads((OUT / 'callback_report.json').read_text(encoding='utf8'))
        assert report['passed']
        return dict(tests=report['tests'], real_ros_run=False,
                    detail='Actual production callbacks with ROS interface test doubles in a separate process; no DDS or vehicle simulator')
    check('offline_callback_contracts', callback_contracts)

    def goal_chain_contracts():
        result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tools',
                               '-p','test_offline_goal_chain_replay.py'],cwd=ROOT,
                              capture_output=True,encoding='utf8',errors='replace',timeout=30)
        (OUT/'test_offline_goal_chain_replay.log').write_text(result.stdout+result.stderr,encoding='utf8')
        if result.returncode:raise AssertionError('Supplied goal chain failed; see test_offline_goal_chain_replay.log')
        return dict(tests=4,scope='Synthetic supplied perception, plans and motion; not field acceptance')
    check('offline_goal_chain_contracts',goal_chain_contracts)

    def kinematic_replay():
        fixture = ROOT / 'test_artifacts/server_validation_20260930/full_town_attempt6.json'
        if not fixture.is_file():
            warning('saved_town_ackermann_replay',
                    'Historical Town fixture is unavailable; no replay was run.')
            return
        result = subprocess.run([sys.executable, str(ROOT / 'tools/offline_follow_path_replay.py')],
                                cwd=ROOT, capture_output=True, encoding='utf8', errors='replace', timeout=30)
        (OUT / 'kinematic_replay.log').write_text(result.stdout + result.stderr, encoding='utf8')
        replay_path = OUT / 'kinematic_replay.json'
        if not replay_path.is_file():
            warning('saved_town_ackermann_replay',
                    'Historical Ackermann replay could not produce a result; see kinematic_replay.log.')
            return
        report = json.loads(replay_path.read_text(encoding='utf8'))
        modified = [r for r in report['runs'] if r['split'] and r['corrected_steering']
                    and not r.get('planning_radius_cap_baseline', False)]
        detail = (f"Historical SBR Ackermann replay only; model cases passed="
                  f"{report['modified_model_cases_passed']} (runner exit {result.returncode}), completed="
                  f"{sum(bool(r['completed']) for r in modified)}/{len(modified)}, "
                  f"max cross-track={max(r['max_cross_track_error'] for r in modified):.3f} m. "
                  'This bicycle-model fixture does not validate the active ideal-point carrier, ROS, or BeamNG.')
        # Keep this old-vehicle replay as a diagnostic artifact, but do not use
        # it as a gate for the point-carrier profile selected by this revision.
        warning('saved_town_ackermann_replay', detail)
    kinematic_replay()

    config = yaml.safe_load((ROOT / "nav2_adapter/config/nav2_official_control.yaml").read_text(encoding="utf8"))
    def contracts():
        launch = ast.parse((ROOT / "nav2_adapter/launch/nav2_official_control.launch.py").read_text(encoding="utf8"))
        executables = [kw.value.value for n in ast.walk(launch) if isinstance(n, ast.Call)
                       for kw in n.keywords if kw.arg == "executable" and isinstance(kw.value, ast.Constant)]
        assert "odom_tf_broadcaster" not in executables, "duplicate TF publisher in official stack"
        params_node = next(n for n in ast.walk(launch)
                           if isinstance(n, ast.Dict) and any(
                               isinstance(k, ast.Constant) and k.value == "input_topic" for k in n.keys))
        geometry = ast.parse((ROOT / "nav2_adapter/wildos_nav2_adapter/vehicle_geometry.py").read_text(encoding="utf8"))
        obstacle_max_height = next(ast.literal_eval(n.value) for n in ast.walk(geometry)
            if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name)
            and t.id == "SBR_OBSTACLE_MAX_HEIGHT" for t in n.targets))
        params = {}
        for key, value in zip(params_node.keys, params_node.values):
            try:
                key_value = ast.literal_eval(key)
                if isinstance(value, ast.Name) and value.id == "SBR_OBSTACLE_MAX_HEIGHT":
                    params[key_value] = obstacle_max_height
                else:
                    params[key_value] = ast.literal_eval(value)
            except (ValueError, TypeError):
                # LaunchConfiguration-backed carrier settings are asserted
                # below from their declared launch defaults instead.
                continue
        node_tree = ast.parse((ROOT / "nav2_adapter/wildos_nav2_adapter/cloud_filter.py").read_text(encoding="utf8"))
        defaults = {n.args[0].value: ast.literal_eval(n.args[1]) for n in ast.walk(node_tree)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == "declare_parameter"}
        defaults.update(params)
        assert defaults["input_topic"] == "/spot1/ouster/front/points_filtered"
        assert defaults["output_frame"] == "spot1/base_link"
        assert defaults["ground_plane_radius"] >= 6.0
        assert 0 <= defaults["min_height"] < defaults["max_height"]
        legacy_launch = (ROOT / "nav2_adapter/launch/nav2_control.launch.py").read_text(encoding="utf8")
        legacy_config = yaml.safe_load((ROOT / "nav2_adapter/config/nav2_control.yaml").read_text(encoding="utf8"))
        assert '"clearing_topic": "/wildos/clearing_points"' in legacy_launch
        for name in ("local_costmap", "global_costmap"):
            legacy_layer = legacy_config[name][name]["ros__parameters"]["obstacle_layer"]
            assert set(legacy_layer["observation_sources"].split()) == {"cloud", "clearing_cloud"}
            assert legacy_layer["cloud"]["marking"] and not legacy_layer["cloud"]["clearing"]
            assert legacy_layer["clearing_cloud"]["clearing"] and not legacy_layer["clearing_cloud"]["marking"]
        wildos_config = yaml.safe_load((ROOT / "official_adapter/config/beamng_wildos.yaml").read_text(encoding="utf8"))
        triangulation_config = yaml.safe_load((ROOT / "official_adapter/config/beamng_triangulation.yaml").read_text(encoding="utf8"))
        assert math.isclose(wildos_config["syncsub_slop"], 0.05)
        wildos_source = (VIS / "visual_navigation/wildos/nav.py").read_text(encoding="utf8")
        assert '"syncsub_slop": 0.05' in wildos_source
        assert math.isclose(triangulation_config["syncsub_slop"], 0.1)
        assert math.isclose(triangulation_config["camera_mask_max_skew_sec"], 0.05)
        assert wildos_config["syncsub_queue_size"] <= 10
        queue_size = int(triangulation_config["syncsub_queue_size"])
        # BeamNG.tech's documented 20 Hz default × 3.6 s inference delay,
        # including one boundary sample. Runtime sensor overrides are unknown.
        required_lidar_history = math.ceil(20.0 * 3.6) + 1
        assert queue_size >= required_lidar_history
        triangulator_source = (VIS / "visual_navigation/explorfm_triangulation/obj_mask_triangulation.py").read_text(encoding="utf8")
        assert "camera_mask_timestamp_mismatch" in triangulator_source
        assert "mask_lidar_timestamp_mismatch" in triangulator_source
        assert "camera_info_stamps_ns" in triangulator_source
        for name in ("local_costmap", "global_costmap"):
            cm = config[name][name]["ros__parameters"]
            layer = cm["obstacle_layer"]
            assert cm["global_frame"] == "spot1/odom"
            assert cm["robot_base_frame"] == "spot1/nav_base_link"
            assert cm["rolling_window"] and "static_layer" not in cm["plugins"]
            assert cm["track_unknown_space"] is False
            # A point-carrier profile must still update after clear/reset. In
            # Humble, disabled bounds providers + active inflation can leave
            # inflation non-current forever, blocking PlannerServer.
            assert layer["enabled"] is True
            assert layer["footprint_clearing_enabled"] is True
            assert set(layer["observation_sources"].split()) == {"cloud", "clearing_cloud"}
            assert layer["cloud"]["topic"] == defaults["output_topic"]
            assert layer["clearing_cloud"]["topic"] == params["clearing_topic"]
            assert layer["cloud"]["marking"] and not layer["cloud"]["clearing"]
            assert layer["clearing_cloud"]["clearing"] and not layer["clearing_cloud"]["marking"]
            for source in (layer, layer["cloud"], layer["clearing_cloud"]):
                # Town absolute Z must not be clipped by default 2 m bounds.
                assert source["min_obstacle_height"] < -100.0
                assert source["max_obstacle_height"] > 100.0
            assert "footprint" not in cm
            assert math.isclose(cm["robot_radius"], 0.15)
            # Humble RPP uses sin((resolution/2)/radius) for rotation time.
            # The former 1 cm proxy gave a negative timestep and unbounded loop.
            ratio = cm["resolution"] / (2.0 * cm["robot_radius"])
            assert 0.0 < ratio <= 1.0 and 2.0 * math.sin(ratio) > 0.0
            assert cm["inflation_layer"]["inflation_radius"] >= cm["robot_radius"]
        ctrl_tree = ast.parse((ROOT / "nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py").read_text(encoding="utf8"))
        source_tree = ast.parse((ROOT / 'nav2_adapter/wildos_nav2_adapter/source_control.py').read_text(encoding='utf8'))
        constants = {n.targets[0].id: ast.literal_eval(n.value) for n in source_tree.body
                     if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                     and n.targets[0].id == 'CONTROLLER_FRAME'}
        ctrl = literal_parameter_defaults(ctrl_tree, constants)
        assert ctrl['cmd_vel_frame'] == 'spot1/nav_base_link'
        bridge_tree = ast.parse((ROOT / "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py").read_text(encoding="utf8"))
        bridge = {}
        for node in ast.walk(bridge_tree):
            if (not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute)
                    or node.func.attr != "declare_parameter" or len(node.args) < 2):
                continue
            try:
                bridge[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
            except (ValueError, TypeError):
                pass
        desired = config["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"]
        assert config["controller_server"]["ros__parameters"]["FollowPath"]["use_collision_detection"] is True
        assert desired >= 1.2 and ctrl["max_speed"] == 0.0
        assert ctrl["overspeed_limit"] == 0.0
        radius = ctrl["wheelbase"] / math.tan(ctrl["max_steering"] * ctrl["max_steer_angle"])
        planner_params = config["planner_server"]["ros__parameters"]
        assert planner_params["planner_plugins"] == ["GridBased"]
        assert "CarLike" not in planner_params
        assert planner_params["GridBased"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"
        controller = config["controller_server"]["ros__parameters"]
        assert controller["FollowPath"]["use_rotate_to_heading"]
        assert not controller["FollowPath"]["allow_reversing"]
        assert "maneuver_goal_checker" in controller["goal_checker_plugins"]
        assert 0 < controller["maneuver_goal_checker"]["xy_goal_tolerance"] <= 0.6
        assert controller["maneuver_goal_checker"]["yaw_goal_tolerance"] < 0.4
        assert not controller["maneuver_goal_checker"]["stateful"]
        assert controller["odom_topic"] == "/spot1/odom"
        assert controller["FollowPath"]["min_lookahead_dist"] >= 3.0
        assert bridge["carrier_model"] == "ideal_point"
        assert bridge["planner_id"] == ""
        assert bridge["minimum_turning_radius"] == 0.0
        kinematic_source = (ROOT / "nav2_adapter/wildos_nav2_adapter/kinematic_vehicle_control.py").read_text(encoding="utf8")
        kinematic_tree = ast.parse(kinematic_source)
        kinematic_class = next(n for n in kinematic_tree.body
                               if isinstance(n, ast.ClassDef) and n.name == "KinematicVehicleControl")
        kinematic_methods = {n.name: n for n in kinematic_class.body if isinstance(n, ast.FunctionDef)}
        tick_source = ast.get_source_segment(kinematic_source, kinematic_methods["tick"])
        parameter_source = ast.get_source_segment(kinematic_source, kinematic_methods["on_parameters"])
        assert "queue_lua_command" not in tick_source
        assert "self.rpc_worker.snapshot" in tick_source
        assert "self.rpc_worker.set_command" in tick_source
        assert "self.rpc_worker.set_active(state_ok)" in tick_source
        assert "self.rpc_worker.retry()" in parameter_source
        assert "KinematicRpcWorker" in kinematic_source
        init_source = ast.get_source_segment(kinematic_source, kinematic_methods["__init__"])
        exchange_source = ast.get_source_segment(kinematic_source, kinematic_methods["exchange"])
        assert init_source.index("self.rpc_worker.set_active(False)") < init_source.index("self.rpc_worker.start()")
        assert "install_fail_closed_reads(PrefixedLengthSocket)" in init_source
        assert "BeamNGpy(" not in init_source and "queue_lua_command" not in init_source
        assert "BeamNGpy(" in exchange_source and "self.disconnect_beamng()" in exchange_source
        assert "self.lua_started = True" in exchange_source
        point_launch_source = (ROOT / "nav2_adapter/launch/kinematic_ideal_point.launch.py").read_text(encoding="utf8")
        point_launch = ast.parse(point_launch_source)
        assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                   and n.func.id == "Node" and any(
                       kw.arg == "executable" and isinstance(kw.value, ast.Constant)
                       and kw.value.value == "kinematic_vehicle_control" for kw in n.keywords)
                   for n in ast.walk(point_launch))
        point_parameters = next(n for n in ast.walk(point_launch) if isinstance(n, ast.Dict)
                                and any(isinstance(k, ast.Constant) and k.value == "carrier_model"
                                        for k in n.keys))
        point_defaults = {ast.literal_eval(k): ast.literal_eval(v)
                          for k, v in zip(point_parameters.keys, point_parameters.values)}
        assert point_defaults == {"enable": False, "carrier_model": "ideal_point",
                                  "minimum_turning_radius": 0.0, "max_angular_rate": 0.8}
        nav_launch = ast.parse((ROOT / "nav2_adapter/launch/nav2_official_control.launch.py").read_text(encoding="utf8"))
        nav_defaults = {}
        for node in ast.walk(nav_launch):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == "DeclareLaunchArgument" and node.args
                    and isinstance(node.args[0], ast.Constant)):
                default = next((kw.value for kw in node.keywords if kw.arg == "default_value"), None)
                if isinstance(default, ast.Constant):
                    nav_defaults[node.args[0].value] = default.value
        assert nav_defaults["footprint_front_extent"] == "0.0"
        assert nav_defaults["footprint_rear_extent"] == "0.0"
        assert nav_defaults["footprint_half_width"] == "0.0"
        assert nav_defaults["filter_footprint_returns"] == "false"
        graph_launch = ast.parse((ROOT / "official_adapter/launch/official_graph_stack.launch.py").read_text(encoding="utf8"))
        graph_defaults = {}
        for node in ast.walk(graph_launch):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == "DeclareLaunchArgument" and node.args
                    and isinstance(node.args[0], ast.Constant)):
                default = next((kw.value for kw in node.keywords if kw.arg == "default_value"), None)
                if isinstance(default, ast.Constant):
                    graph_defaults[node.args[0].value] = default.value
        assert graph_defaults["carrier_model"] == "ideal_point"
        assert graph_defaults["state_topic"] == "/wildos/kinematic_state"
        standoff_source = (ROOT / "official_adapter/beamng_wildos_adapter/goal_standoff.py").read_text(encoding="utf8")
        standoff_tree = ast.parse(standoff_source)
        standoff_class = next(n for n in standoff_tree.body
                              if isinstance(n, ast.ClassDef) and n.name == "GoalStandoff")
        standoff_defaults = {}
        for node in ast.walk(standoff_class):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "declare_parameter" and len(node.args) == 2):
                try:
                    standoff_defaults[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
                except (ValueError, TypeError):
                    pass
        assert triangulation_config["navigation_goal_topic"] == "/spot1/raw_object_waypoint"
        assert "input_topic" not in standoff_defaults
        assert standoff_defaults["output_topic"] == "/spot1/imgnav_waypoint"
        assert graph_defaults["planner_goal_topic"] == "/spot1/imgnav_waypoint"
        assert "create_subscription(String, '/spot1/object_localization'" in standoff_source
        assert "create_publisher(String, '/spot1/object_confirmation'" in standoff_source
        field_test = (ROOT / "test_artifacts/server_validation_20261006/green_house_station_arrival_kinematic_test.py").read_text(encoding="utf8")
        assert '"/spot1/raw_object_waypoint"' in field_test
        assert "legacy PoseStamped beside its" in field_test
        assert "GoalStandoff consumes object_localization" in field_test
        assert 'data.get("mapped_stamp_ns", data.get("mask_stamp_ns"))' in field_test
        assert '"output_clock", data.get(' in field_test and '"clock"' in field_test
        trial_runner = (ROOT / "test_artifacts/server_validation_20261006/run_green_house_station_arrival_kinematic_test.py").read_text(encoding="utf8")
        assert "preflight.json" in trial_runner
        assert '"/wildos_graph_path_bridge", "carrier_model", "ideal_point"' in trial_runner
        assert '"/wildos_graph_path_bridge", "minimum_turning_radius", "0.0"' in trial_runner
        assert '"/planner_server", "planner_plugins", "GridBased"' in trial_runner
        assert 'require_counts("/vehicles/ego/sensors/state", 1, 1)' in trial_runner
        assert 'require_counts("/wildos/kinematic_state", 1, 1)' in trial_runner
        assert "keep_disabled(\"/wildos_graph_path_bridge\", \"follow_path\")" in trial_runner
        assert "kinematic_vehicle_control" in trial_runner
        bridge_source = (ROOT / "nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py").read_text(encoding="utf8")
        assert 'create_subscription(PoseStamped, "/spot1/imgnav_waypoint"' in bridge_source
        assert 'create_subscription(PoseStamped, "/spot1/remembered_object_goal"' in bridge_source
        assert 'create_subscription(String, "/spot1/object_confirmation"' in bridge_source
        ackermann = yaml.safe_load((ROOT / "nav2_adapter/config/nav2_ackermann_control.yaml").read_text(encoding="utf8"))
        ackermann_controller = ackermann["controller_server"]["ros__parameters"]["FollowPath"]
        assert not ackermann_controller["use_rotate_to_heading"]
        assert ackermann_controller["allow_reversing"]
        assert math.isclose(ackermann["planner_server"]["ros__parameters"]["CarLike"]["minimum_turning_radius"], 12.0)
        from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH, SBR_WHEELBASE
        demo = (ROOT / "tools/official_wildos_window_demo.sh").read_text(encoding="utf8")
        assert f"footprint_front_extent:={SBR_FRONT_EXTENT:g}" in demo
        assert f"footprint_rear_extent:={SBR_REAR_EXTENT:g}" in demo
        assert f"footprint_half_width:={SBR_HALF_WIDTH:g}" in demo
        assert "filter_footprint_returns:=true" in demo
        assert "state_topic:=/vehicles/ego/sensors/state" in demo
        assert math.isclose(ctrl["wheelbase"], SBR_WHEELBASE)
        global_cm = config["global_costmap"]["global_costmap"]["ros__parameters"]
        return dict(desired_speed=desired, legacy_vehicle_radius=radius,
                    active_carrier_model=bridge["carrier_model"],
                    active_planner="GridBased", active_min_turn_radius=0.0,
                    object_goal_pipeline=["object_localization", "object_confirmation/imgnav_waypoint", "GraphPathBridge"],
                    raw_triangulation_waypoint="diagnostic_only_unconsumed",
                    ideal_point_launch="kinematic_ideal_point.launch.py",
                    kinematic_rpc_off_ros_executor=True,
                    ideal_point_grid_proxy_radius=0.15, physical_demo_keeps_sbr_footprint=True,
                    triangulation_queue_size=queue_size,
                    triangulation_queue_required_at_20hz=required_lidar_history,
                    wildos_odom_camera_slop_sec=wildos_config["syncsub_slop"],
                    triangulator_mask_lidar_slop_sec=triangulation_config["syncsub_slop"],
                    camera_mask_max_skew_sec=triangulation_config["camera_mask_max_skew_sec"],
                    obstacle_avoidance_disabled=False,
                    legacy_clearing_stream=True, sensor_sync_slop_seconds=0.05,
                    limitation="Checks configured defaults; runtime rates, sync drops, and measured vehicle steering remain unverified.")
    check("configuration_contracts", contracts)

    def runtime_entry():
        script=(ROOT / "tools/official_wildos_window_demo.sh").read_text(encoding="utf8")
        launch=(ROOT / "nav2_adapter/launch/nav2_official_control.launch.py").read_text(encoding="utf8")
        config=yaml.safe_load((ROOT / "official_adapter/config/beamng_wildos.yaml").read_text(encoding="utf8"))
        default_map=re.search(r'^MAP_MODE=\$\{MAP_MODE:-([^}]+)\}', script, re.M)
        default_speed=re.search(r'^MOTION_SPEED=\$\{MOTION_SPEED:-([^}]+)\}', script, re.M)
        assert default_map and default_speed
        assert 'nav2_official_control.launch.py' in script
        assert 'graph_path_bridge' in script
        assert 'planner_goal_topic:=/spot1/graphnav_destination' in script
        assert '--memory_in_process_only' in script
        assert '-p follow_path:=false' in script
        assert 'ros2 param set /wildos_graph_path_bridge follow_path true' in script
        assert 'visual_navigation.wildos.nav' in script
        assert 'clearing_topic' in launch
        assert float(default_speed.group(1)) >= 1.2
        hardcoded_ackermann = ('carrier_model:=ackermann' in script
                               and 'planner_id:=CarLike' in script
                               and 'minimum_turning_radius:=8.0' in script)
        return dict(default_map=default_map.group(1), default_speed_mps=float(default_speed.group(1)),
                    configured_first_query=config["object_search_config"]["text_queries"],
                    town_scenario_in_entry="east_coast_usa" in script,
                    hardcoded_ackermann=hardcoded_ackermann)
    check("runtime_entry_contract", runtime_entry)
    entry=next(row for row in results if row["name"] == "runtime_entry_contract")
    if entry["status"] == "PASS" and not entry["detail"]["town_scenario_in_entry"]:
        warning("town_launch_gap", "official_wildos_window_demo.sh defaults to american_road/sawmill and has no East Coast Town scenario. Its successful run cannot validate the requested Town spawn. Use a Town scenario and verify odom within 3 m before recording a Town result.")
    if entry["status"] == "PASS" and entry["detail"]["hardcoded_ackermann"]:
        warning("legacy_ackermann_entry", "official_wildos_window_demo.sh hardcodes carrier_model=ackermann, planner_id=CarLike, and an 8 m minimum radius. It is a physical-car demo and cannot verify the ideal-point, zero-radius acceptance profile; run the dedicated ideal-point setup and require runtime status before movement.")
    if entry["status"] == "PASS" and entry["detail"]["configured_first_query"] != ["green residential house"]:
        warning("target_sequence_gap", "The generic demo does not validate the requested unseeded sequence. Use run_green_house_station_arrival_kinematic_test.py with the ideal-point profile: search green residential house first, switch to gas station only after confirmed arrival, retain passive memory in one process, and never preseed the station.")
    historical=(ROOT / "east_coast_region/generated/test_20260927/双区域测试报告.md")
    if historical.is_file():
        history=historical.read_text(encoding="utf8")
        if "车辆保持静止" in history and "未执行闭环驾驶或 Nav2" in history:
            warning("historical_town_scope", "The Sep 27 Town/Sawmill report verified sensors, graph scoring and graph paths while stationary. It contains no closed-loop driving, Nav2 costmap or FollowPath evidence.")

    def synthetic():
        # A deliberately simplified one-dimensional ray model, NOT Nav2.
        pts = np.array([[8.,0.,-.3], [0.,0.,.7], [8.,0.,4.]], dtype=np.float32)
        marked, cleared = split_costmap_points(pts, min_height=.1, max_height=3.,
            filter_footprint=True, front_extent=2.3, rear_extent=2.3,
            half_width=1., voxel=.2, max_points=200)
        old_occupied = {4}
        new_occupied = old_occupied.copy()
        for x,y,z in cleared:
            new_occupied.difference_update(range(0, int(x)))
        assert len(marked) == 0 and old_occupied == {4} and not new_occupied
        # Inverse pose transforms must use the same pose, even at nonzero altitude.
        angle=.4
        rot=np.array([[math.cos(angle),-math.sin(angle),0], [math.sin(angle),math.cos(angle),0],[0,0,1]])
        position=np.array([734.294,-11.946,52.154])
        world=np.array([[740.,-10.,52.8]])
        body=(world-position) @ rot
        np.testing.assert_allclose(body @ rot.T + position, world, atol=1e-9)
        wrong=body @ rot.T + position + np.array([.24,0,0])
        assert np.linalg.norm(wrong-world) > .2
        return dict(old_ghost_cells=1, new_ghost_cells=0, pose_mismatch_example_m=.24,
                    limitation="Synthetic ray model proves the missing-endpoint mechanism, not actual Nav2 collision behavior.")
    check("synthetic_clearing_and_transform", synthetic)

    def fixture():
        directory=ROOT / "sensor_calibration/captures/5bd8a8d980704fdd82c55c6a68a73097"
        meta=json.loads((directory / "lidar.json").read_text(encoding="utf8"))
        assert meta["frame_observed"] == "raw_world" and meta["pose_stable"]
        raw=np.load(directory / "lidar.npy", allow_pickle=False)
        assert raw.ndim == 2 and raw.shape[1] == 3 and np.isfinite(raw).all()
        pc=load(ROOT / "official_adapter/beamng_wildos_adapter/pointcloud.py", "offline_adapter_cloud")
        sampled=pc.limit_points(raw,30000)
        pose=meta["metadata"]["pose_before"]
        # Fixture metadata lacks an exact up vector: explicitly assume upright.
        forward=np.asarray(pose["direction"],dtype=float)
        forward/=np.linalg.norm(forward)
        left=np.cross([0.,0.,1.],forward)
        left/=np.linalg.norm(left)
        rotation=np.column_stack((forward,left,np.cross(forward,left)))
        body=(sampled-np.asarray(pose["position"])) @ rotation
        body=body[np.linalg.norm(body,axis=1)<=40.]
        marked,cleared=split_costmap_points(body,min_height=.1,max_height=3.,filter_footprint=True,
            front_extent=2.3,rear_extent=2.3,half_width=1.,voxel=.2,max_points=20000)
        assert len(marked)<=20000 and len(cleared)<=20000
        assert np.isfinite(marked).all() and np.isfinite(cleared).all()
        return dict(raw_points=len(raw),sampled_points=len(sampled),marking_points=len(marked),
                    clearing_points=len(cleared),
                    limitation="Different static calibration fixture, assumed upright pose; no synchronized stamps. Not Town failure replay or calibrated obstacle classification.")
    check("fixture_pointcloud_smoke", fixture)

    def region():
        build=load(ROOT / "east_coast_region/build.py", "offline_region")
        poly=build.CONFIG["regions"]["eca_town_industrial"]["polygon"]
        data=[]
        for name,x,y in [("town_spawn",734.294,-11.946),("previous_failure",789.,-19.93)]:
            dist=min(build.distance_to_segment(x,y,*a,*b) for a,b in zip(poly,poly[1:]+poly[:1]))
            data.append(dict(name=name, xy=[x,y], boundary_distance_m=dist, retained_point=build.point_kept(x,y)))
        return dict(positions=data, warning="Town spawn is on the region boundary. Outside-region does not imply collision; roads may cross the polygon.")
    check("region_geometry", region)

    def syntax():
        files=list((ROOT / "official_adapter").rglob("*.py"))+list((ROOT / "nav2_adapter").rglob("*.py"))
        files += list((VIS / "visual_navigation").rglob("*.py"))
        files += list((VIS / "test").glob("*.py"))
        files += [ROOT / 'tools' / name for name in ('official_planner_probe.py',
                  'offline_callback_check.py', 'offline_follow_path_replay.py',
                  'diagnose_town_follow_path.py', 'offline_navigation_check.py', 'package_official_bundle.py',
                  'offline_interface_replay.py', 'test_offline_interface_replay.py',
                  'patch_beamng_sensor_lifecycle.py')]
        files += [ROOT / 'test_artifacts/server_validation_20261002/full_town_memory_test.py',
                  ROOT / 'test_artifacts/offline_navigation/write_mechanism_repair_report.py']
        for path in files: ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        return dict(files=len(files))
    check("python_syntax", syntax)

    def bundle():
        path=ROOT / "official_wildos_beamng_bundle.zip"
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest.upper() == (ROOT / "official_wildos_beamng_bundle.sha256").read_text().split()[0].upper()
        with zipfile.ZipFile(path) as z:
            assert z.testzip() is None
            for name in z.namelist():
                if not name.endswith("/"):
                    assert z.read(name) == (ROOT / name).read_bytes(), f"stale bundle: {name}"
            assert "nav2_adapter/wildos_nav2_adapter/drive_math.py" in z.namelist()
            assert "wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner_node.cpp" in z.namelist()
            manifest = json.loads(path.with_suffix('.manifest.json').read_text(encoding='utf8'))
            assert manifest['sha256'] == digest
            assert set(manifest['members']) == set(z.namelist())
            for name, expected in manifest['members'].items():
                assert hashlib.sha256(z.read(name)).hexdigest() == expected, name
            from package_official_bundle import bundle_sources
            assert bundle_sources(ROOT, path) <= set(z.namelist()), 'missing current source'
        return dict(sha256=digest)
    check("deployment_bundle", bundle)
    pending=["ROS lifecycle, QoS, TF availability and timestamp freshness",
             "East Coast Town scenario entry and actual birth pose; current official demo defaults to American Road/sawmill",
             "Nav2 ObstacleLayer/InflationLayer and Smac/FollowPath actual execution",
             "Real ROS/DDS action integration for ordered motion sections and cusp goal checker; callback contracts pass with interface doubles",
             "C++ graph mapper/planner compile and test_route_mechanism in a ROS 2 build environment",
             "Exact Town prefix/costmap replay is available; full vehicle execution still requires runtime verification",
             "Active ideal-point carrier: command-to-measured translation/yaw response and stable FollowPath tracking; physical-car steering/torque checks belong to the separate Ackermann profile",
             "Same-process multi-goal search speed comparison with memory off/on"]
    report=dict(checks=results, pending_runtime=pending,
                scope="Offline source/default-config/unit/synthetic checks only; not a driving acceptance result.")
    (OUT / "report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
    lines=["# 离线导航检查结果", "", "此报告不代表 Nav2 或 BeamNG 现场验收通过。", "", "| 检查 | 结果 | 详情 |", "|---|---|---|"]
    for row in results:
        detail=json.dumps(row["detail"],ensure_ascii=False).replace("|","/")
        lines.append(f"| {row['name']} | {row['status']} | {detail} |")
    lines += ["", "## 待运行环境验证", ""] + ["- " + p for p in pending]
    (OUT / "report.md").write_text("\n".join(lines)+"\n",encoding="utf8")
    for row in results: print(row["status"], row["name"], row["detail"])
    print("Report:", OUT / "report.md")
    return int(any(r["status"]=="FAIL" for r in results))

if __name__ == "__main__":
    raise SystemExit(main())
