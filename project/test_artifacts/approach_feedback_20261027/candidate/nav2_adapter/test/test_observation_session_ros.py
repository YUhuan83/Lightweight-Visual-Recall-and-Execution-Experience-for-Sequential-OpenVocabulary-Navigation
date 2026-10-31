import unittest
from copy import deepcopy
from unittest.mock import Mock
import test_continuous_exploration_ros as fixtures
import test_graph_path_bridge_routing as paths
from wildos_nav2_adapter.input_quality import stamp_ns

class ObservationSessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixtures.ContinuousExplorationTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixtures.ContinuousExplorationTests.tearDownClass()
    def setUp(self):
        self.f=fixtures.ContinuousExplorationTests();self.f.setUp();self.n=self.f.moving()
        self.addCleanup(self.n.destroy_node);n=self.n;n.full_path_execution=True
        p=deepcopy(n.motion_queue[0]);n.follow_active['path_stamp_ns']=stamp_ns(p)
        for pose in p.poses:pose.header.stamp=n.get_clock().now().to_msg()
        n.on_controller_progress(p);n.on_exploration_retired(self.f.retirement())
        self.assertIsNotNone(n.retired_motion)
        n.visible_hint=dict(query=n.query,point=[8.,0.],bearing=0.,navigation_heading=0.,observation_stamp_ns=n.get_clock().now().nanoseconds)
        n.relay_destination=Mock();self.assertTrue(n.activate_visible_approach())
        self.new=paths.route((0.,0.),(8.,0.))
        n.route_metadata[stamp_ns(self.new)]=dict(exploration_id=None,complete=True,goal=[8.,0.],mode='known')
    def test_fresh_visible_destination_can_replace_retired_exploration_in_background(self):
        self.n.route=self.new
        self.assertTrue(self.n.retired_replacement_context())
    def test_live_route_plans_before_retired_exploration_finishes(self):
        n=self.n;n.start_segment=Mock();handle=n.follow_handle
        n.on_route(self.new)
        self.assertIs(n.latest_route,self.new)
        n.start_segment.assert_called_once_with(continuous=True)
        self.assertIs(n.follow_handle,handle);handle.cancel_goal_async.assert_not_called()
    def test_stale_mismatched_or_retired_route_cannot_claim_visible_session(self):
        n=self.n;n.route=self.new
        key=stamp_ns(self.new);meta=n.route_metadata[key]
        for invalid in ('old_mask','wrong_query','wrong_destination','old_route'):
            saved=deepcopy(n.visible_hint);saved_meta=deepcopy(meta);stamp=deepcopy(self.new.header.stamp)
            if invalid=='old_mask':n.visible_hint['observation_stamp_ns']-=3_000_000_000
            elif invalid=='wrong_query':n.visible_hint['query']='gas station'
            elif invalid=='wrong_destination':meta['goal']=[80.,0.]
            else:self.new.header.stamp.sec-=10
            self.assertFalse(n.retired_replacement_context(),invalid)
            n.visible_hint=saved;n.route_metadata[key]=saved_meta;meta=n.route_metadata[key];self.new.header.stamp=stamp
    def test_confirmation_memory_or_query_reset_cannot_borrow_this_handoff(self):
        n=self.n;n.route=self.new
        for field,value in (('active_object_confirmation',{'instance_id':'verified'}),('memory_goal',(8.,0.)),('query_reset_pending',True),('object_found',True)):
            old=getattr(n,field);setattr(n,field,value)
            self.assertFalse(n.retired_replacement_context(),field);setattr(n,field,old)
    def test_unknown_or_retired_route_identity_is_not_a_new_visible_goal(self):
        n=self.n;n.route=self.new;meta=n.route_metadata[stamp_ns(self.new)]
        meta['exploration_id']='retired-other';n.retired_explorations['retired-other']=None
        self.assertFalse(n.retired_replacement_context())

if __name__=='__main__':unittest.main()
