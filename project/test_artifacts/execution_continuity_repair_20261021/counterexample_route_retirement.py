"""Expected failing handoff cases matching the closed 100-second retirement race."""
import json
import unittest
import rclpy
from std_msgs.msg import String
from wildos_nav2_adapter.input_quality import stamp_ns
import test_exploration_cost_ros as comparison
from test_graph_path_bridge_routing import drivable_route


class RetiredOwnerHandoffCounterexamples(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.fixture=comparison.ExplorationCostTests();self.fixture.setUp();self.node=self.fixture.node
    def tearDown(self):self.fixture.tearDown()
    def replacement(self):
        n=self.node;path=drivable_route((0.,0.),(16.,0.))
        path.header.stamp=n.get_clock().now().to_msg()
        path.header.stamp.nanosec+=1_000_000
        if path.header.stamp.nanosec>=1_000_000_000:
            path.header.stamp.sec+=1;path.header.stamp.nanosec-=1_000_000_000
        n.route_metadata[stamp_ns(path)]=dict(exploration_id='c:3',complete=True,
            goal=[16.,0.],observation_heading=0.,vehicle_cost_selected=False)
        return path
    def retire(self):
        self.node.on_exploration_retired(String(data=json.dumps(dict(
            exploration_id='a:1',reason='frontier_resolved',
            stamp_ns=self.node.get_clock().now().nanoseconds))))
    def test_replacement_received_during_choice_wait_must_survive_old_retirement(self):
        self.fixture.compare()
        path=self.replacement();self.node.on_route(path)
        self.retire()
        self.assertTrue(stamp_ns(self.node.latest_route)==stamp_ns(path)
                        or stamp_ns(self.node.route)==stamp_ns(path),
                        'Replacement was dropped; must wait for its next graph publication.')
    def test_deferred_new_owner_must_survive_old_comparison_and_retirement(self):
        n=self.node;n.start_segment()
        self.fixture.response(drivable_route((0.,0.),(8.,0.),(3.,0.),(12.,0.)))
        path=self.replacement();n.on_route(path)
        self.assertIs(n.latest_route,path)
        self.fixture.response(drivable_route((0.,0.),(14.,0.)))
        self.retire()
        self.assertTrue(stamp_ns(n.latest_route)==stamp_ns(path)
                        or stamp_ns(n.route)==stamp_ns(path),
                        'Retiring original owner erased an already deferred new-owner route.')
