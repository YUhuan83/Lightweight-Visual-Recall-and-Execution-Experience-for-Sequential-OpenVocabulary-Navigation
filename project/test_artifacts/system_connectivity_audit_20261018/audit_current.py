"""Read-only contract audit. Gap probes passing mean a defect remains present."""
from pathlib import Path
import hashlib
import importlib.util
import io
import json
import sys
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'nav2_adapter'),
               str(ROOT/'nav2_adapter/test'), str(ROOT/'official_adapter'),
               str(ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
spec = importlib.util.spec_from_file_location('historical_gaps',
    ROOT/'test_artifacts/whole_flow_audit_20261017/audit_readonly.py')
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
from test_navigation_transactions_ros import NavigationTransactionTests
from test_graph_path_bridge_routing import route

manifest_path = ROOT/'deploy/20261018_smoothing_boundary_candidate/official_wildos_beamng_bundle.manifest.json'
manifest = json.loads(manifest_path.read_text(encoding='utf8'))


def snapshot():
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            for name in manifest['members']}


OBSERVATIONS = {}


class ResetConnectivityGaps(unittest.TestCase):
    def test_successful_clear_history_blocks_live_graph_routes(self):
        fixture = NavigationTransactionTests(); fixture.setUp()
        try:
            node = fixture.node
            responses = fixture.service()
            responses[-1].set_result(S(success=True))
            steps = []
            for index in range(3):
                node.expected = (10.+index, 0.)
                before = len(responses)
                node.release_visible_approach('visible_target_route_timeout')
                dispatched = len(responses) > before
                if dispatched:
                    responses[-1].set_result(S(success=True))
                steps.append(dict(dispatched=dispatched,
                                  attempts=node.graph_reset_attempts,
                                  pending=node.query_reset_pending,
                                  waiting=len(node.graph_reset_waiting)))
            self.assertEqual([s['dispatched'] for s in steps], [True, True, False])
            self.assertTrue(node.query_reset_pending)
            self.assertEqual(node.graph_reset_waiting, {})
            self.assertEqual(node.last_status['reason'], 'response_budget_exhausted')
            node.route_diagnostic = Mock()
            incoming = route((0., 0.), (6., 0.))
            incoming.header.stamp = node.get_clock().now().to_msg()
            node.on_route(incoming)
            node.route_diagnostic.assert_called_once()
            self.assertEqual(node.route_diagnostic.call_args.args[0], 'query_reset_pending')
            OBSERVATIONS['successful_clear_blocks_graph'] = dict(
                steps=steps, all_dispatched_services_succeeded=True,
                live_route_rejection='query_reset_pending',
                scope='Production Python callbacks with ROS doubles, no vehicle/DDS')
        finally:
            fixture.tearDown()


def run_group(label, suite):
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    (OUT/(label+'.log')).write_text(stream.getvalue(), encoding='utf8')
    return dict(label=label, tests=result.testsRun, failures=len(result.failures),
                errors=len(result.errors), skipped=len(result.skipped),
                expected_behavior_reproduced=result.wasSuccessful())


def main():
    before = snapshot()
    selected = (
        'test_heading_negative_evidence_is_shared_maximum',
        'test_success_only_execution_samples_always_retrieve_neutral_prior',
        'test_sensor_restart_clock_and_control_epoch_disagree',
        'test_late_large_clock_rollback_is_indistinguishable_from_restart',
        'test_successful_short_sections_have_no_overall_progress_exit')
    gaps = unittest.TestSuite([old.WholeFlowGaps(name) for name in selected])
    gaps.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(ResetConnectivityGaps))
    gap_result = run_group('confirmed_gap_probes', gaps)
    guards = unittest.defaultTestLoader.loadTestsFromTestCase(NavigationTransactionTests)
    guard_result = run_group('existing_transaction_guards', guards)
    after = snapshot()
    differences = [name for name, want in manifest['members'].items() if before[name] != want]
    report = dict(scope=__doc__, frozen_candidate=manifest['sha256'],
        members=len(before), current_matches_frozen=not differences,
        differences_from_frozen=differences, production_unchanged=before==after,
        results=[gap_result, guard_result],
        current_gap_observations={**old.OBSERVATIONS, **OBSERVATIONS},
        dds_verified=False, physics_verified=False, complete_navigation_passed=False,
        limits='Gap probes assert known bad behavior, not repair acceptance. Existing guards use ROS doubles. Static contract findings and previous field evidence are separate.')
    (OUT/'source_sha256.json').write_text(json.dumps(before, indent=2)+'\n', encoding='utf8')
    (OUT/'current_audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))
    return 0 if (before==after and not differences and
                 gap_result['expected_behavior_reproduced'] and
                 guard_result['expected_behavior_reproduced']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
