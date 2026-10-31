import sys,unittest
from pathlib import Path
from copy import deepcopy
from unittest.mock import Mock
root=Path(sys.argv.pop(1));sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
import test_continuous_exploration_ros as fixtures
import test_graph_path_bridge_routing as paths
from wildos_nav2_adapter.input_quality import stamp_ns

class RetiredVisibleContract(unittest.TestCase):
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
    def test_fresh_visible_destination_is_a_valid_background_replacement(self):
        self.n.route=self.new
        self.assertTrue(self.n.retired_replacement_context())
    def test_live_route_plans_before_retired_exploration_finishes(self):
        n=self.n;n.start_segment=Mock();handle=n.follow_handle
        n.on_route(self.new)
        self.assertIs(n.latest_route,self.new)
        n.start_segment.assert_called_once_with(continuous=True)
        self.assertIs(n.follow_handle,handle);handle.cancel_goal_async.assert_not_called()

unittest.main(verbosity=2)
