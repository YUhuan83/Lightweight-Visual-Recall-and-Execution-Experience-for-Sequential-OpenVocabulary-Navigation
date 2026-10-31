"""Do not turn a persistent new surface into either old-instance evidence or a lockout."""
from copy import deepcopy
import json
from pathlib import Path
from unittest.mock import Mock
import unittest

from std_msgs.msg import String
import test_surface_association_ros as surface_tests


class InstanceHandoffTests(unittest.TestCase):
    setUp = surface_tests.SurfaceAssociationTests.setUp
    tearDown = surface_tests.SurfaceAssociationTests.tearDown
    advance = surface_tests.SurfaceAssociationTests.advance
    localize = surface_tests.SurfaceAssociationTests.localize
    rejected = surface_tests.SurfaceAssociationTests.rejected

    def lock(self):
        for _ in range(5):
            self.localize()
        self.anchor = deepcopy(self.goal.confirmed_instance_anchor)
        self.bridge.active_object_confirmation = deepcopy(self.anchor)
        self.bridge.upstream_instance_id = self.anchor['instance_id']
        self.bridge.expected = self.bridge.confirmed_route_goal = (12., 0.)
        self.bridge.confirmed_route_object = (20., 0.)

    def last_message(self):
        return self.goal.confirmation_pub.publish.call_args.args[0]

    def test_consistent_new_surface_requests_release_without_relabeling_old_instance(self):
        self.lock()
        for _ in range(4):
            self.localize(40.)
        data = json.loads(self.last_message().data)
        self.assertEqual(data['instance_id'], self.anchor['instance_id'])
        self.assertFalse(data['search_confirmed'])
        self.assertIn('replacement_candidate', data)
        self.assertEqual(data['replacement_candidate']['inliers'], 4)
        self.assertEqual(data['replacement_candidate']['position'], [40., 0., 2.])
        self.assertEqual(self.goal.confirmed_instance_anchor, self.anchor)
        self.bridge.on_object_confirmation(self.last_message())
        self.assertIsNotNone(self.bridge.instance_release_pending)
        self.assertEqual(self.bridge.instance_release_pending['instance_id'], self.anchor['instance_id'])
        self.assertFalse(self.bridge.object_found)
        self.assertIsNone(self.bridge.expected)
        self.assertTrue(self.bridge.query_reset_pending)

    def test_one_or_disjoint_new_surface_does_not_cancel_current_path(self):
        self.lock()
        self.bridge.reset = Mock()
        for x in (40., 50., 60., 70.):
            self.localize(x)
            self.assertNotIn('replacement_candidate', json.loads(self.last_message().data))
            self.bridge.on_object_confirmation(self.last_message())
        self.bridge.reset.assert_not_called()

    def test_current_instance_reobservation_breaks_alternative_streak(self):
        self.lock()
        for _ in range(3):
            self.localize(40.)
        self.localize(20.)
        self.localize(40.)
        self.assertNotIn('replacement_candidate', json.loads(self.last_message().data))

    def test_ambiguous_scan_cannot_help_confirm_alternative(self):
        self.lock()
        for _ in range(3):
            self.localize(40.)
        self.localize(localization_valid=False, reason='ambiguous_depth_surfaces')
        self.localize(40.)
        self.assertNotIn('replacement_candidate', json.loads(self.last_message().data))

    def test_release_ack_clears_lock_and_requires_fresh_new_confirmation(self):
        self.lock()
        for _ in range(4):
            self.localize(40.)
        self.bridge.on_object_confirmation(self.last_message())
        request = self.bridge.instance_release_pending
        self.assertIsNotNone(request)
        self.goal.on_instance_release(String(data=json.dumps(request)))
        self.assertIsNone(self.goal.confirmed_instance_anchor)
        self.assertFalse(self.goal.observations)
        self.assertFalse(self.goal.replacement_observations)
        self.assertEqual(request['reason'], 'instance_superseded')
        self.localize(20.)
        self.assertFalse(self.goal.observations)  # Existing bounded region backoff prevents ping-pong.
        self.bridge.on_instance_release_ack(String(data=json.dumps(dict(request, released=True))))
        self.assertIsNone(self.bridge.instance_release_pending)
        for _ in range(4):
            self.localize(40.)
        new = json.loads(self.last_message().data)
        self.assertTrue(new['search_confirmed'])
        self.assertNotEqual(new['instance_id'], self.anchor['instance_id'])
        self.assertFalse(self.bridge.object_found)

    def test_recorded_v43_new_surface_can_form_independent_candidate(self):
        data = json.loads((Path(__file__).parent/'fixtures/v43_instance_binding.json').read_text())
        self.goal.active_query = data['anchor']['query']
        self.goal.query_started_ns = 0
        self.goal.last_localization_ns = data['anchor']['observation_stamp_ns']
        self.goal.confirmed_instance_anchor = deepcopy(data['anchor'])
        outputs = []
        for observation in data['localizations']:
            self.now = observation['observation_stamp_ns'] + 100_000_000
            self.goal.on_localization(String(data=json.dumps(observation)))
            if self.goal.confirmation_pub.publish.call_args:
                outputs.append(json.loads(self.last_message().data))
        replacements = [x for x in outputs if x.get('replacement_candidate')]
        self.assertTrue(replacements)
        first = replacements[0]
        self.assertLess(first['observation_stamp_ns'], data['first_mismatch_stamp_ns']+10_000_000_000)
        self.assertGreater(first['replacement_candidate']['inliers'], 3)
        self.assertFalse(first['search_confirmed'])

    def test_malformed_other_instance_or_old_candidate_cannot_release(self):
        self.lock()
        self.bridge.reset = Mock()
        for changes in (dict(instance_id='other'), dict(replacement_candidate={'inliers': 1}),
                        dict(observation_stamp_ns=self.anchor['observation_stamp_ns']-1)):
            msg = self.rejected(instance_id=self.anchor['instance_id'], reason='instance_mismatch')
            data = json.loads(msg.data)
            data.update(changes)
            self.bridge.on_object_confirmation(String(data=json.dumps(data)))
        self.bridge.reset.assert_not_called()

    def test_alternative_is_not_a_memory_region_override_or_escape_cancel(self):
        self.lock()
        for _ in range(4):
            self.localize(40.)
        message = json.loads(self.last_message().data)
        self.bridge.memory_active = {'id': 'remembered-view'}
        self.assertFalse(self.bridge.replacement_requires_release(message))
        self.bridge.memory_active = None
        self.bridge.execution_recovery_owns_motion = Mock(return_value=True)
        self.assertFalse(self.bridge.replacement_requires_release(message))

    def test_duplicate_release_event_does_not_restart_stop_transaction(self):
        self.lock()
        for _ in range(4):
            self.localize(40.)
        message = self.last_message()
        self.bridge.on_object_confirmation(message)
        request = deepcopy(self.bridge.instance_release_pending)
        epoch = self.bridge.epoch
        self.bridge.on_object_confirmation(message)
        self.assertEqual(self.bridge.epoch, epoch)
        self.assertEqual(self.bridge.instance_release_pending, request)

    def test_query_change_and_expiry_discard_alternative_history(self):
        self.lock()
        for _ in range(3):
            self.localize(40.)
        self.now += int((self.goal.confirmation_window_sec+1)*1e9)
        self.localize(40.)
        self.assertNotIn('replacement_candidate', json.loads(self.last_message().data))
        self.goal.on_query(String(data='station'))
        self.assertFalse(self.goal.replacement_observations)


if __name__ == '__main__':
    unittest.main()
