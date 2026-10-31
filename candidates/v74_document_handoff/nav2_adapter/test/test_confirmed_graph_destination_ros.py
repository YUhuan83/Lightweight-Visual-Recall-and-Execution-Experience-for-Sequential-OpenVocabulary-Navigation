"""A confirmed far target is a GraphNav destination, not another 8m task."""
import json,math,unittest,time
from types import SimpleNamespace as S
from copy import deepcopy
from std_msgs.msg import String
import test_confirmed_progress_ros as progress


class ConfirmedGraphDestinationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):progress.ConfirmedProgressTests.setUpClass()
    @classmethod
    def tearDownClass(cls):progress.ConfirmedProgressTests.tearDownClass()
    def setUp(self):
        self.f=progress.ConfirmedProgressTests();self.f.setUp();self.addCleanup(self.f.tearDown)
        self.n=self.f.node;original=self.n.get_parameter
        self.n.get_parameter=lambda name:S(value=True) if name=='graph_destination_approach' else original(name)
    def test_far_confirmed_target_remains_far_graph_destination(self):
        target,_=self.f.pending((40.,0.,2.));self.n.publish_standoff(target)
        self.assertGreater(self.n.last_goal[0],20.)
        self.assertAlmostEqual(math.dist(self.n.last_goal,(40.,0.)),self.n.approach_distance()[0])
        self.assertFalse(self.n.last_standoff_clamped)
    def test_real_confirmation_handoff_keeps_far_destination_without_claiming_arrival(self):
        self.f.pending((40.,0.,2.));self.n.retry_confirmation(force=True)
        msg=self.n.confirmation_pub.publish.call_args.args[0];d=json.loads(msg.data)
        self.assertTrue(d['approach_goal_ready']);self.assertGreater(d['standoff'][0],20.)
        bridge=self.f.fixture.bridge;bridge.carrier_model='ackermann'
        bridge.odom_received_at=time.monotonic()
        bridge.on_object_confirmation(msg)
        self.assertEqual(bridge.expected,tuple(d['standoff']));self.assertFalse(bridge.object_found)
    def test_global_destination_does_not_require_local_map_covering_remote_object(self):
        target,_=self.f.pending((70.,0.,2.));self.n.publish_standoff(target)
        self.assertGreater(self.n.last_goal[0],50.)
        self.assertIsNone(self.n.vehicle_grid)
    def test_small_measurement_jitter_does_not_replace_far_destination(self):
        target,_=self.f.pending((40.,0.,2.));self.n.publish_standoff(target)
        previous=self.n.last_goal;count=self.n.publisher.publish.call_count
        target.pose.position.x+=.2;self.n.publish_standoff(target)
        self.assertEqual(self.n.last_goal,previous);self.assertEqual(self.n.publisher.publish.call_count,count)
    def test_nearby_target_still_uses_existing_observable_pose_selection(self):
        target,_=self.f.pending((12.,0.,2.));self.n.publish_standoff(target)
        enabled=self.n.last_goal
        old=self.n.get_parameter
        self.n.get_parameter=lambda name:S(value=False) if name=='graph_destination_approach' else old(name)
        self.n.last_goal=None;self.n.publish_standoff(target)
        self.assertEqual(self.n.last_goal,enabled)
    def test_query_switch_does_not_reuse_old_far_destination(self):
        target,_=self.f.pending((40.,0.,2.));self.n.publish_standoff(target)
        self.f.fixture.query('gas station');self.assertIsNone(self.n.last_goal)
        target,_=self.f.pending((0.,40.,2.));self.n.publish_standoff(target)
        self.assertGreater(self.n.last_goal[1],20.);self.assertAlmostEqual(self.n.last_goal[0],0.)
