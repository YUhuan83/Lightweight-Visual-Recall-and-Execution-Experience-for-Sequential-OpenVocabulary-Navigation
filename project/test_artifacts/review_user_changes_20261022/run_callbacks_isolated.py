"""Run repository callback suites without overwriting the shared test report."""
import importlib.util
import io
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "nav2_adapter"), str(ROOT / "nav2_adapter/test"),
    str(ROOT / "official_adapter"),
    str(ROOT / "wildos_port/repro_check/src/nebula2-wildos/visual_navigation")]

SUITES = (
    "test_execution_continuity_review_ros", "test_reachable_observation_ros",
    "test_exploration_cost_ros", "test_graph_path_bridge_routing",
    "test_planning_phase_ros", "test_cmd_vel_to_control_ros",
    "test_controller_source_ros", "test_goal_confirmation_ros",
    "test_recorded_memory_handoff_ros", "test_execution_mechanism_ros",
    "test_multi_goal_session_ros", "test_sensor_observation_lifecycle_ros",
    "test_execution_transport_health_ros", "test_confirmed_recovery_ros",
    "test_committed_frontier_path_ros", "test_planner_execution_ownership_ros",
    "test_navigation_transactions_ros", "test_execution_progress_ros",
    "test_motion_intent_handoff_ros", "test_surface_association_ros",
    "test_short_endpoint_handoff_ros", "test_tracking_handoff_ros",
    "test_triangulation_query_scope", "test_native_vehicle_control_ros",
    "test_source_control_ros", "test_wheel_contact", "test_fail_closed_transport",
    "test_direct_vehicle_control", "test_vehicle_control_lease",
    "test_kinematic_rpc_worker", "test_kinematic_profile_status",
    "test_visible_approach_ros", "test_arrival_criteria",
    "test_visualization_bridge_ros")


def main(label):
    from offline_ros_fakes import install
    install()
    output = Path(__file__).resolve().parent / "callback_runs" / label
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for name in SUITES:
        folder = (ROOT / "wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test"
                  if name == "test_triangulation_query_scope" else ROOT / "nav2_adapter/test")
        spec = importlib.util.spec_from_file_location(name, folder / (name + ".py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        stream = io.StringIO()
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromModule(module))
        (output / (name + ".log")).write_text(stream.getvalue(), encoding="utf-8")
        row = dict(suite=name, tests=result.testsRun, failures=len(result.failures),
                   errors=len(result.errors), skipped=len(result.skipped), passed=result.wasSuccessful())
        results.append(row)
        print(json.dumps(row))
        if not result.wasSuccessful():
            print(stream.getvalue())
    report = dict(label=label, suites=results,
                  tests=sum(item["tests"] for item in results),
                  passed=all(item["passed"] for item in results),
                  real_ros_run=False, vehicle_simulation_run=False)
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
