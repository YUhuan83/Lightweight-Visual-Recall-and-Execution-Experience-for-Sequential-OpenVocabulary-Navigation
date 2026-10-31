import copy
import unittest
from unittest.mock import Mock
from wildos_nav2_adapter.input_quality import stamp_ns
import test_full_path_execution_ros as full
import test_continuous_exploration_ros as continuous


class RetirementBoundaryTests(unittest.TestCase):
    setUpClass=classmethod(lambda cls:full.FullPathExecutionTests.setUpClass())
    tearDownClass=classmethod(lambda cls:full.FullPathExecutionTests.tearDownClass())
    setUp=full.FullPathExecutionTests.setUp
    mixed=full.FullPathExecutionTests.mixed

    def moving(self):
        helper=continuous.ContinuousExplorationTests();helper.node=self.node
        n=helper.moving();n.full_path_execution=True;n.retirement_pub=Mock()
        p,_,_=self.mixed();n.motion_queue=[p]
        n.follow_active['path_stamp_ns']=stamp_ns(p)
        progress=copy.deepcopy(p)
        first=next(i for i,v in enumerate(p.poses) if v.pose.position.x==2.)
        progress.poses=progress.poses[:first+1]
        for v in progress.poses:v.header.stamp=n.get_clock().now().to_msg()
        n.on_controller_progress(progress)
        return n,helper,progress

    def check_fence(self,n,p):
        n.retirement_pub.publish.assert_called_once()
        actual=n.retirement_pub.publish.call_args.args[0]
        self.assertEqual(stamp_ns(actual),stamp_ns(p))
        self.assertEqual(actual.header.frame_id,p.header.frame_id)
        for field in ('x','y','z'):
            self.assertEqual(getattr(actual.poses[-1].pose.position,field),
                             getattr(p.poses[-1].pose.position,field))
        for field in ('x','y','z','w'):
            self.assertEqual(getattr(actual.poses[-1].pose.orientation,field),
                             getattr(p.poses[-1].pose.orientation,field))
        self.assertEqual(actual.poses[-1].pose.position.x,2.)

    def test_intent_change_revokes_the_hidden_reverse_suffix(self):
        n,_,p=self.moving();handle=n.follow_handle
        self.assertTrue(n.prepare_intent_change())
        self.check_fence(n,p)
        self.assertTrue(n.following);handle.cancel_goal_async.assert_not_called()

    def test_frontier_retirement_revokes_the_hidden_reverse_suffix(self):
        n,helper,p=self.moving()
        n.on_exploration_retired(helper.retirement())
        self.assertIsNotNone(n.retired_motion)
        self.check_fence(n,p)

    def test_direct_retirement_also_restricts_the_controller(self):
        n,_,p=self.moving()
        self.assertTrue(n.finish_current_exploration_section('frontier-A'))
        self.check_fence(n,p)

    def test_stale_progress_cannot_be_used_to_retain_an_old_intent(self):
        n,_,_=self.moving();n._controller_path_received_at-=2.
        self.assertFalse(n.prepare_intent_change())
        n.retirement_pub.publish.assert_not_called()

    def test_legacy_execution_needs_no_controller_retirement_message(self):
        n,_,_=self.moving();n.full_path_execution=False
        self.assertTrue(n.prepare_intent_change())
        n.retirement_pub.publish.assert_not_called()

    def test_failed_retirement_does_not_revoke_healthy_motion(self):
        n,_,_=self.moving();n.follow_request=object()
        self.assertFalse(n.finish_current_exploration_section('frontier-A'))
        n.retirement_pub.publish.assert_not_called()

    def test_new_route_also_restricts_old_motion_while_replacement_plans(self):
        n,helper,p=self.moving();n.start_segment=Mock()
        n.on_route(helper.route((20.,0.),'frontier-B'))
        self.check_fence(n,p)
        self.assertTrue(n.following)

    def test_duplicate_retirement_cannot_extend_the_original_boundary(self):
        n,_,p=self.moving()
        self.assertTrue(n.prepare_intent_change())
        later=copy.deepcopy(p);later.poses[-1].pose.position.x=4.
        n.on_controller_progress(later)
        self.assertTrue(n.restrict_current_direction())
        self.check_fence(n,p)

if __name__=='__main__':unittest.main()
