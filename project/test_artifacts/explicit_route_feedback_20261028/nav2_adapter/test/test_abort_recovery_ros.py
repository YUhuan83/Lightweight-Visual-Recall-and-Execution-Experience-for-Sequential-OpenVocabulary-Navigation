import unittest
from unittest.mock import Mock
import rclpy
import test_graph_path_bridge_routing as fixtures
from wildos_nav2_adapter.input_quality import stamp_ns


class ExplorationAbortRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        fixtures.GraphBridgeRoutingTests.setUp(self)
        self.addCleanup(self.node.destroy_node)
        n=self.node;n.follow_enabled=True;n.following=True
        n.route=n.latest_route=fixtures.drivable_route((0.,0.),(10.,0.))
        n.route_metadata[stamp_ns(n.route)]=dict(exploration_id='frontier:1',goal=[20.,0.],complete=False)
        n.route_endpoint=(10.,0.);n.recovery_attempts=2
        n.try_execution_escape=Mock(return_value=False)
        n.fail_confirmed_approach=Mock(return_value=False)
        n.finish_execution_feedback=Mock()
        n.route_rejection_pub=Mock()

    def test_third_actual_abort_rejects_the_unproductive_frontier(self):
        self.node.follow_failed('follow_aborted',action_status=6)
        self.node.route_rejection_pub.publish.assert_called_once()
        p=self.node.route_rejection_pub.publish.call_args.args[0]
        self.assertEqual(stamp_ns(p),stamp_ns(self.node.route))
        self.assertEqual((p.pose.position.x,p.pose.position.y),(10.,0.))

    def test_repeated_completed_prefix_does_not_restore_abort_credit(self):
        n=self.node;n.sync_recovery_progress()
        n.credit_recovery_progress()
        self.assertEqual(n.recovery_attempts,2)
        n.follow_failed('follow_aborted',action_status=6)
        n.route_rejection_pub.publish.assert_called_once()

    def test_first_abort_still_allows_existing_recovery(self):
        self.node.recovery_attempts=0
        self.node.follow_failed('follow_aborted',action_status=6)
        self.node.route_rejection_pub.publish.assert_not_called()

    def test_actual_new_progress_keeps_the_existing_credit_rule(self):
        n=self.node;n.sync_recovery_progress();n.odom.pose.pose.position.x=1.1
        n.credit_recovery_progress();self.assertEqual(n.recovery_attempts,0)
        n.follow_failed('follow_aborted',action_status=6)
        n.route_rejection_pub.publish.assert_not_called()

    def test_cancelled_action_is_not_an_aborted_frontier(self):
        self.node.follow_failed('follow_aborted',action_status=5)
        self.node.route_rejection_pub.publish.assert_not_called()

    def test_explicit_object_and_memory_goals_are_not_frontiers(self):
        n=self.node;n.expected=(20.,0.)
        n.follow_failed('follow_aborted',action_status=6)
        n.route_rejection_pub.publish.assert_not_called()

    def test_retired_motion_uses_its_existing_owner_barrier(self):
        n=self.node;n.retired_motion=dict(identity='old:1');n.stop_retired_motion=Mock()
        n.follow_failed('follow_aborted',action_status=6)
        n.stop_retired_motion.assert_called_once_with('follow_aborted',action_status=6)
        n.route_rejection_pub.publish.assert_not_called()

if __name__=='__main__':unittest.main()
