"""Near approach proposals use the controller's existing obstacle map too."""
import unittest
from copy import deepcopy
import test_visible_approach_ros as fixture
from nav_msgs.msg import OccupancyGrid


class ApproachLocalMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.VisibleApproachTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.VisibleApproachTests.tearDownClass()
    def setUp(self):
        self.fixture=fixture.VisibleApproachTests();self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        self.node=n=self.fixture.perception
        g=OccupancyGrid();g.header.frame_id='spot1/odom';g.header.stamp=self.fixture.stamp()
        g.info.width=g.info.height=160;g.info.resolution=.25
        g.info.origin.position.x=g.info.origin.position.y=-20.;g.info.origin.orientation.w=1.
        g.data=[0]*(160*160);n.on_vehicle_grid(g)
        self.local=deepcopy(g);self.local.info.resolution=.2
        self.local.info.width=self.local.info.height=80
        self.local.info.origin.position.x=self.local.info.origin.position.y=-8.
        self.local.data=[0]*(80*80)
    def block(self,x,y):
        i=int((x+8.)/.2);j=int((y+8.)/.2)
        self.local.data[j*80+i]=100
    def deliver(self,message=None):
        message=self.local if message is None else message
        import inspect
        if 'local' in inspect.signature(self.node.on_vehicle_grid).parameters:
            self.node.on_vehicle_grid(message,local=True)
        else:
            # Old version has no local-map callback: restore the same input
            # checkpoint so the selection-method difference can be tested.
            self.node.local_vehicle_grid=deepcopy(message)
    def accept(self,point=(2.5,0.)):
        return self.node.observation_pose_filter(bearing=0.)(point,0.)
    def test_known_local_body_obstacle_cannot_be_selected_on_free_global_map(self):
        self.block(5.8,.2);self.deliver();self.assertFalse(self.accept())
    def test_local_center_obstacle_cannot_be_selected(self):
        self.block(2.5,0.);self.deliver();self.assertFalse(self.accept())
    def test_clear_maps_keep_normal_forward_pose(self):
        self.deliver();self.assertTrue(self.accept())
    def test_new_clearing_grid_restores_pose_without_stale_obstacle(self):
        self.block(5.8,.2);self.deliver();self.assertFalse(self.accept())
        self.fixture.now+=100_000_000;self.local.header.stamp=self.fixture.stamp()
        self.local.data=[0]*6400;self.deliver();self.assertTrue(self.accept())
    def test_out_of_order_grid_cannot_replace_new_obstacle(self):
        old=deepcopy(self.local);self.fixture.now+=100_000_000
        self.local.header.stamp=self.fixture.stamp();self.block(5.8,.2);self.deliver()
        self.deliver(old);self.assertFalse(self.accept())
    def test_far_destination_remains_a_global_map_decision(self):
        self.deliver();self.assertTrue(self.accept((12.,0.)))
    def test_partial_local_coverage_does_not_create_an_artificial_map_wall(self):
        self.deliver();self.assertTrue(self.accept((7.,0.)))
    def test_other_frame_does_not_enter_vehicle_map_filter(self):
        self.block(5.8,.2);self.local.header.frame_id='spot1/base_link';self.deliver()
        self.assertTrue(self.accept())
    def test_ideal_carrier_keeps_its_existing_point_navigation_contract(self):
        original=self.node.get_parameter
        from types import SimpleNamespace as S
        self.node.get_parameter=lambda key:S(value='ideal_point') if key=='carrier_model' else original(key)
        self.block(5.8,.2);self.deliver();self.assertTrue(self.accept())
