"""Original 3D goal drives navigation; confirmation remains arrival evidence."""
import ast,json,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from unittest.mock import Mock,patch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import test_body_arrival_ros as body

class UpstreamTargetApproachTests(unittest.TestCase):
    def setUp(self):
        self.f=body.BodyArrivalTests();self.f.setUp();self.addCleanup(self.f.tearDown)
        self.b=self.f.bridge;self.b.upstream_target_approach=True
        self.b.expected=None;self.b.waypoint_pub=Mock()
        self.b.query_reset_pending=False
    def goal(self,x=20.,y=0.,z=2.):
        p=PoseStamped();p.header.frame_id='spot1/odom'
        p.header.stamp=self.b.get_clock().now().to_msg()
        p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=z
        p.pose.orientation.w=1.;return p
    def test_raw_3d_goal_reaches_existing_destination_before_consensus(self):
        p=self.goal();self.b.on_waypoint(p)
        self.assertEqual(self.b.expected,(20.,0.))
        sent=self.b.waypoint_pub.publish.call_args.args[0]
        self.assertEqual(sent.pose.position.z,2.)
        self.assertIsNone(self.b.confirmed_route_goal)
        self.assertFalse(self.b.object_found)
    def test_runtime_mode_subscribes_to_the_original_raw_goal_topic(self):
        cls=type(self.b);parameter=cls.get_parameter;subscribe=cls.create_subscription;topics=[]
        def get(node,name):
            return S(value=True) if name=='upstream_target_approach' else parameter(node,name)
        def sub(node,kind,topic,*args,**kwargs):
            topics.append(topic);return subscribe(node,kind,topic,*args,**kwargs)
        with patch.object(cls,'get_parameter',get),patch.object(cls,'create_subscription',sub):
            n=cls()
        try:
            self.assertIn('/spot1/raw_object_waypoint',topics)
            self.assertNotIn('/spot1/imgnav_waypoint',topics)
            self.assertTrue(n.upstream_target_approach)
        finally:n.destroy_node()
    def test_actual_triangulator_publisher_flows_through_real_destination_callback(self):
        source=Path(__file__).resolve().parents[2]/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
        tree=ast.parse(source.read_text(encoding='utf8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObjectMaskTriangulator')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_navigation_goal_and_marker')
        namespace=dict(np=np,PoseStamped=PoseStamped,String=String,Marker=Marker,json=json)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(source),'exec'),namespace)
        publisher=Mock();publisher.publish.side_effect=self.b.on_waypoint
        sender=S(triangulated_position=np.array([20.,0.,2.]),
            localization_quality=dict(association_validated=True,confidence=.8),
            global_frame='spot1/odom',get_clock=self.b.get_clock,get_logger=Mock(),
            nav_goal_publisher=publisher,localization_pub=Mock(),triangulated_obj_publisher=Mock(),
            active_query='house',found_lidar_in_mask=True,bearing_views=[],
            current_observation=S(header=self.goal().header,odom=self.b.odom))
        namespace['publish_navigation_goal_and_marker'](sender)
        self.assertEqual(self.b.expected,(20.,0.))
        self.assertEqual(self.b.waypoint_pub.publish.call_args.args[0].pose.position.z,2.)
        self.assertIsNone(self.b.confirmed_route_goal);self.assertFalse(self.b.object_found)
    def test_later_confirmation_cannot_replace_raw_goal_with_standoff(self):
        self.b.on_waypoint(self.goal())
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=[8.,0.]))
        self.b.route_confirmed_target.assert_not_called()
        self.assertEqual(self.b.expected,(20.,0.))
        self.assertIsNone(self.b.confirmed_route_goal)
        self.assertIsNone(self.b.deferred_confirmed_approach)
    def test_first_confirmed_observation_replaces_active_provisional_raw_goal(self):
        self.b.on_waypoint(self.goal(x=20.))
        self.b.following=True
        handle=Mock();self.b.follow_handle=handle
        self.b.on_object_confirmation(self.f.confirmation(point=(40.,0.,2.),standoff=None))
        self.assertEqual(self.b.expected,(40.,0.))
        self.assertEqual(self.b.waypoint_pub.publish.call_args.args[0].pose.position.x,40.)
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.b.object_found)
    def test_invalid_or_stale_confirmation_cannot_replace_provisional_goal(self):
        self.b.on_waypoint(self.goal(x=20.));self.b.following=True
        self.b.on_object_confirmation(self.f.confirmation(point=(40.,0.,2.),
            quality=dict(confidence=.9,association_validated=False)))
        self.b.on_object_confirmation(self.f.confirmation(point=(40.,0.,2.),
            observation_stamp_ns=self.f.fixture.now-6_000_000_000))
        self.assertEqual(self.b.expected,(20.,0.))
    def test_already_confirmed_instance_is_not_preempted_by_another_confirmation(self):
        self.b.on_waypoint(self.goal(x=20.))
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=None))
        self.b.following=True
        self.b.on_object_confirmation(self.f.confirmation(point=(40.,0.,2.),standoff=None,instance_id='house-b'))
        self.assertEqual(self.b.expected,(20.,0.))
    def test_idle_raw_outlier_cannot_replace_confirmed_target_before_rejection(self):
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=None))
        self.b.following=False;self.b.pending=False
        epoch=self.b.epoch
        self.b.on_waypoint(self.goal(x=40.))
        self.assertEqual(self.b.expected,(20.,0.));self.assertEqual(self.b.epoch,epoch)
    def test_idle_other_instance_cannot_replace_target_or_finish_query(self):
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=None))
        self.b.following=False;self.b.pending=False
        self.b.on_object_confirmation(self.f.confirmation(point=(5.4,0.,2.),standoff=None,instance_id='house-b'))
        self.assertEqual(self.b.expected,(20.,0.));self.assertFalse(self.b.object_found)
    def test_matching_confirmation_can_refine_idle_target(self):
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=None))
        self.b.following=False;self.b.pending=False
        self.b.on_object_confirmation(self.f.confirmation(point=(24.,0.,2.),standoff=None))
        self.assertEqual(self.b.expected,(24.,0.));self.assertFalse(self.b.object_found)
    def test_route_end_does_not_start_added_release_timer(self):
        self.b.on_waypoint(self.goal());self.b.await_object_observation()
        self.assertIsNone(self.b.observation_wait_until)
        self.assertEqual(self.b.expected,(20.,0.));self.assertFalse(self.b.object_found)
    def test_missing_standoff_cannot_prevent_original_goal_navigation(self):
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=None))
        self.assertEqual(self.b.expected,(20.,0.));self.b.route_confirmed_target.assert_not_called()
        self.assertFalse(self.b.object_found)
    def test_fresh_body_arrival_is_still_required(self):
        self.b.on_object_confirmation(self.f.confirmation(point=(5.401,0.,2.),standoff=None))
        self.assertFalse(self.b.object_found)
        self.b.on_object_confirmation(self.f.confirmation(point=(5.4,0.,2.),standoff=None))
        self.assertTrue(self.b.object_found)
    def test_delayed_confirmation_cannot_mark_arrival(self):
        message=self.f.confirmation(point=(5.4,0.,2.))
        self.f.fixture.now+=6_000_000_000
        self.b.on_object_confirmation(message);self.assertFalse(self.b.object_found)
    def test_carried_geometry_cannot_replace_live_goal_or_mark_arrival(self):
        self.b.on_waypoint(self.goal())
        m=self.f.confirmation(point=(5.4,0.,2.),approach_refresh=True,
            base_observation_stamp_ns=self.f.fixture.now,approach_update_stamp_ns=self.f.fixture.now)
        self.b.active_object_confirmation=dict(query='house',object_position=[5.4,0.],observation_stamp_ns=self.f.fixture.now)
        self.b.on_object_confirmation(m)
        self.assertEqual(self.b.expected,(20.,0.));self.assertFalse(self.b.object_found)
    def test_memory_comparison_keeps_control_until_a_view_is_selected(self):
        self.b.memory_selecting=True;self.b.on_waypoint(self.goal())
        self.assertIsNone(self.b.expected)
    def test_bearing_step_does_not_replace_original_3d_goal_entry(self):
        self.b.activate_visible_approach=Mock()
        message=String(data=json.dumps(dict(query='house',frame='spot1/odom',
            observation_stamp_ns=self.f.fixture.now,point=[8.,0.],bearing=0.,
            evidence='bearing_only',position_confirmed=False,search_confirmed=False)))
        self.b.on_visible_approach(message)
        self.b.activate_visible_approach.assert_not_called();self.assertIsNone(self.b.visible_hint)
    def test_fresh_matching_memory_observation_hands_off_without_changing_recall_store(self):
        self.b.memory_active=dict(id='view-A',object_position=[20.,0.],object_radius=1.,
            evidence='passive_semantic_landmarks')
        self.b.memory_goal=self.b.expected=(3.,0.)
        self.b.on_waypoint(self.goal())
        self.assertEqual(self.b.expected,(3.,0.))
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=None))
        self.assertEqual(self.b.expected,(20.,0.));self.assertIsNone(self.b.memory_goal)
        self.assertEqual(self.b.memory_active['id'],'view-A')
        self.b.route_confirmed_target.assert_not_called();self.assertFalse(self.b.object_found)
    def test_existing_mode_keeps_its_current_navigation_contract(self):
        self.b.upstream_target_approach=False
        self.b.on_object_confirmation(self.f.confirmation(point=(20.,0.,2.),standoff=[8.,0.]))
        self.b.route_confirmed_target.assert_called_once()
