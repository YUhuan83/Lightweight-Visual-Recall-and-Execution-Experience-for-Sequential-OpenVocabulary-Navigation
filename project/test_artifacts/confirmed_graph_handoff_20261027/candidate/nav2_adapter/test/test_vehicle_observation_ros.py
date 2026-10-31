"""Car observation intentions must fit the actual start, without claiming arrival."""
import json
import math
import unittest
from copy import deepcopy
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import test_visible_approach_ros as fixture
from wildos_nav2_adapter.graph_route import forward_arc_arrival_heading
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge


class VehicleObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): fixture.VisibleApproachTests.setUpClass()
    @classmethod
    def tearDownClass(cls): fixture.VisibleApproachTests.tearDownClass()
    def setUp(self):
        self.f=fixture.VisibleApproachTests()
        with patch.object(fixture,'GraphPathBridge',ContinuousGraphPathBridge):self.f.setUp()
        self.n=self.f.perception;self.b=self.f.bridge
        self.b.carrier_model='ackermann'
        self.b.minimum_turning_radius=12.
        self.n.observation_minimum_turning_radius=12.
        grid=OccupancyGrid();grid.header.frame_id='spot1/odom'
        grid.header.stamp=self.f.stamp();grid.info.width=grid.info.height=100
        grid.info.resolution=.5;grid.info.origin.position.x=grid.info.origin.position.y=-25.
        grid.info.origin.orientation.w=1.;grid.data=[0]*10000
        self.n.on_vehicle_grid(grid)
    def tearDown(self): self.f.tearDown()
    def propose(self,bearing):
        self.n.visible_target=dict(stamp=self.f.now,bearing=bearing)
        self.n.visible_target_count=3
        self.n.publish_visible_step()
        calls=self.n.visible_approach_pub.publish.call_args_list
        return json.loads(calls[-1].args[0].data) if calls else None

    def test_side_bearing_proposes_a_car_feasible_intermediate_pose(self):
        d=self.propose(1.2);self.assertIsNotNone(d)
        heading=forward_arc_arrival_heading((0.,0.),0.,d['point'],12.)
        self.assertIsNotNone(heading)
        self.assertAlmostEqual(heading,d['navigation_heading'])
        self.assertGreater(d['point'][0]*math.cos(1.2)+d['point'][1]*math.sin(1.2),0.)
        self.assertFalse(d['position_confirmed']);self.assertFalse(d['search_confirmed'])

    def test_behind_bearing_does_not_create_a_short_reverse_pose(self):
        self.assertIsNone(self.propose(math.pi))

    def test_expired_vehicle_pose_at_consumer_does_not_start_a_shunt(self):
        self.b.on_visible_approach(String(data=json.dumps(dict(query=self.b.query,frame='spot1/odom',
            observation_stamp_ns=self.f.now,point=[1.,5.],bearing=1.2,navigation_heading=1.2,
            evidence='bearing_only',position_confirmed=False,search_confirmed=False))))
        self.assertFalse(self.b.visible_approach_active)
        self.assertIsNone(self.b.expected)

    def test_nonfinite_navigation_heading_is_rejected(self):
        self.b.on_visible_approach(String(data=json.dumps(dict(query=self.b.query,frame='spot1/odom',
            observation_stamp_ns=self.f.now,point=[8.,0.],bearing=0.,navigation_heading=float('nan'),
            evidence='bearing_only',position_confirmed=False,search_confirmed=False))))
        self.assertIsNone(self.b.visible_hint)

    def test_straight_progress_keeps_bearing_separate_from_navigation_heading(self):
        d=self.propose(.15);self.assertIsNotNone(d)
        self.assertEqual(d['bearing'],.15)
        self.b.on_visible_approach(String(data=json.dumps(d)))
        self.assertTrue(self.b.visible_approach_active)
        self.assertFalse(self.b.object_found)

    def test_no_fresh_global_map_retains_exploration(self):
        self.n.vehicle_grid=None
        self.assertIsNone(self.propose(0.))

    def test_stale_mask_does_not_get_a_new_lifetime(self):
        self.n.visible_target=dict(stamp=self.f.now-3_000_000_000,bearing=.5)
        self.n.visible_target_count=3
        self.n.publish_visible_step()
        self.n.visible_approach_pub.publish.assert_not_called()

    def test_occupied_sweep_does_not_publish_a_point_only_safe_endpoint(self):
        self.n.vehicle_grid.data=[100]*10000
        self.n.vehicle_connectivity=None
        self.assertIsNone(self.propose(.5))

    def test_unknown_terrain_is_not_crossed_by_a_curved_proposal(self):
        self.n.occupancy.data=[-1]*len(self.n.occupancy.data)
        self.assertIsNone(self.propose(.8))

    def test_query_change_clears_vehicle_hint(self):
        d=self.propose(.1);self.b.on_visible_approach(String(data=json.dumps(d)))
        self.f.query('gas station')
        self.assertIsNone(self.b.visible_hint)
        self.assertFalse(self.b.visible_approach_active)

    def active_delayed(self):
        d=self.propose(0.);self.b.on_visible_approach(String(data=json.dumps(d)))
        self.assertTrue(self.b.visible_approach_active)
        # Follow the old exploration leg past the intermediate camera point.
        self.b.odom=deepcopy(self.b.odom)
        self.b.odom.pose.pose.position.x=10.
        self.b.vehicle_pose_feasible=Mock(return_value=True)
        return self.b

    def test_delayed_handoff_replaces_passed_point_with_fresh_forward_proposal(self):
        b=self.active_delayed()
        b.visible_hint.update(point=[18.,0.],observation_stamp_ns=self.f.now)
        b.start_segment()
        self.assertEqual(b.expected,(18.,0.))
        self.assertTrue(b.visible_goal_refresh_pending)
        self.assertTrue(b.last_status['geometry_refreshed'])
        self.assertTrue(b.visible_approach_active)
        self.assertFalse(b.object_found)

    def test_delayed_handoff_without_fresh_forward_point_releases_old_intent(self):
        b=self.active_delayed()
        b.visible_hint['observation_stamp_ns']=self.f.now-3_000_000_000
        b.release_visible_approach=Mock()
        b.start_segment()
        b.release_visible_approach.assert_called_once_with('visible_target_no_vehicle_path')
        self.assertFalse(b.pending)

    def test_still_forward_geometry_does_not_change_the_accepted_intent(self):
        b=self.active_delayed();b.odom.pose.pose.position.x=2.
        previous=b.expected
        self.assertTrue(b.prepare_visible_vehicle_goal())
        self.assertEqual(b.expected,previous)

    def test_confirmed_or_memory_navigation_is_not_replaced_by_vehicle_hint(self):
        b=self.active_delayed();b.visible_approach_active=False
        b.confirmed_route_goal=(8.,0.)
        self.assertTrue(b.prepare_visible_vehicle_goal())
        self.assertEqual(b.expected,(8.,0.))

    def test_refresh_retains_current_action_and_discards_old_unexecuted_suffix(self):
        b=self.active_delayed();b.visible_hint.update(point=[18.,0.],observation_stamp_ns=self.f.now)
        current,tail=Mock(),Mock();handle=Mock();epoch=b.epoch
        b.motion_queue=[current,tail];b.following=True;b.follow_handle=handle
        self.assertFalse(b.prepare_visible_vehicle_goal())
        self.assertEqual(b.motion_queue,[current]);self.assertIs(b.follow_handle,handle)
        self.assertEqual(b.epoch,epoch);self.assertTrue(b.following)
        handle.cancel_goal_async.assert_not_called()

    def test_runtime_contract_requires_the_same_radius_as_actual_smac(self):
        from pathlib import Path
        import yaml
        from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
        root=Path(__file__).parents[1]
        config=yaml.safe_load((root/'config/nav2_ackermann_control.yaml').read_text(encoding='utf8'))
        contract=navigation_runtime_contract(config,'ackermann')
        self.assertEqual(contract['/beamng_vehicle_goal_standoff']['observation_minimum_turning_radius'],
                         contract['/planner_server']['CarLike.minimum_turning_radius'])

    def test_ideal_carrier_retains_original_direction_proposal(self):
        previous=self.n.get_parameter
        self.n.get_parameter=lambda name:S(value='ideal_point') if name=='carrier_model' else previous(name)
        d=self.propose(1.2)
        self.assertNotIn('navigation_heading',d)
        self.assertAlmostEqual(math.atan2(d['point'][1],d['point'][0]),1.2)


if __name__=='__main__':unittest.main()
