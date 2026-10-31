import ast
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'nav2_adapter/test')]
from check_ackermann_cusp_execution import candidate_check,execution_issue,observed_motion_directions,publisher_snapshot,wait_for_publishers
from test_graph_route import grid,set_cell


class CuspProbeContractTests(unittest.TestCase):
    def discovery(self, rows):
        now=[0.];owner=('native','/','std_msgs/msg/String')
        def info(topic):
            return [SimpleNamespace(node_name=n,node_namespace=ns,topic_type=t,endpoint_gid=[i])
                    for i,(n,ns,t) in enumerate(rows(now[0]))]
        def spin(seconds):now[0]+=seconds
        node=SimpleNamespace(get_publishers_info_by_topic=info)
        return wait_for_publishers(node,{'topic':[owner]},spin,clock=lambda:now[0],timeout=2.,stable=.5)

    def test_late_discovery_is_waited_for_and_stably_verified(self):
        snapshot,history,issue=self.discovery(lambda t:[] if t<.6 else [('native','/','std_msgs/msg/String')])
        self.assertIsNone(issue);self.assertGreaterEqual(history[-1]['elapsed'],1.1)
        self.assertEqual(history[0]['publishers']['topic'],[])

    def test_missing_publisher_times_out_without_arming(self):
        _,history,issue=self.discovery(lambda t:[])
        self.assertEqual(issue,'publisher_discovery_timeout');self.assertLessEqual(len(history),21)

    def test_duplicate_or_wrong_owner_is_never_accepted(self):
        for rows in ([('native','/','std_msgs/msg/String')]*2,
                     [('other','/','std_msgs/msg/String')],
                     [('native','/','wrong/type')]):
            with self.subTest(rows=rows):
                _,history,issue=self.discovery(lambda t:rows)
                self.assertEqual(issue,'unexpected_or_duplicate_publisher:topic')
                self.assertEqual(len(history),1)

    def test_disappearing_endpoint_resets_discovery_stability(self):
        _,history,issue=self.discovery(lambda t:[] if .2<t<.6 else [('native','/','std_msgs/msg/String')])
        self.assertIsNone(issue);self.assertGreaterEqual(history[-1]['elapsed'],1.1)
    def test_publisher_snapshot_preserves_absence_duplicates_and_identity(self):
        row=SimpleNamespace(node_name='native',node_namespace='/',topic_type='std_msgs/msg/String',endpoint_gid=[1,2])
        node=SimpleNamespace(get_publishers_info_by_topic=lambda topic:[] if topic=='absent' else [row,row])
        snapshot=publisher_snapshot(node,['absent','duplicate'])
        self.assertEqual(snapshot['absent'],[])
        self.assertEqual(len(snapshot['duplicate']),2)
        self.assertEqual(snapshot['duplicate'][0]['endpoint_gid'],[1,2])
        self.assertEqual(snapshot['duplicate'][0]['node_name'],'native')
    def setUp(self):
        self.points=[(0.,0.),(1.,0.),(2.,0.),(1.,0.),(0.,0.)]
        self.headings=[0.]*5

    def check(self,status=4,global_grid=None,local_grid=None):
        return candidate_check(status,self.points,self.headings,(0.,0.),0.,
            global_grid or grid(),local_grid or grid())

    def test_straight_forward_reverse_is_admissible(self):self.assertIsNone(self.check())

    def test_sub_metre_connector_requires_full_tracking_and_unchanged_checker(self):
        self.points=[(0.,0.),(.5,0.),(0.,0.)];self.headings=[0.]*3
        self.assertIsNone(self.check())
        # Extra lateral offset keeps the target outside the unchanged .2 disk.
        self.points=[(0.,0.),(.5,0.),(0.,.4)]
        self.assertIsNotNone(self.check())

    def test_unsuccessful_planning_and_non_cusp_are_not_armed(self):
        self.assertEqual(self.check(status=6),'planner_not_success')
        self.points=self.points[:3];self.headings=self.headings[:3]
        self.assertEqual(self.check(),'no_cusp')

    def test_obstacle_on_either_map_rejects_candidate(self):
        occupied=grid();set_cell(occupied,3.,.8,100)
        self.assertEqual(self.check(global_grid=occupied),'footprint_collision')
        self.assertIsNotNone(self.check(local_grid=occupied))

    def test_short_cusp_or_excess_curvature_remains_rejected(self):
        self.points=[(0.,0.),(.2,0.),(0.,0.)];self.headings=[0.]*3
        self.assertEqual(self.check(),'outside_component_length')
        self.points=[(0.,0.),(1.,0.),(2.,1.),(1.,0.),(0.,0.)]
        self.headings=[0.,.2,.5,.2,0.]
        self.assertIsNotNone(self.check())

    def test_probe_has_no_video_capture_or_spool_path(self):
        tree=ast.parse((ROOT/'tools/check_ackermann_cusp_execution.py').read_text())
        names={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
        self.assertTrue(names.isdisjoint({'JpegSpool','ScreenCapture','VideoRecorder'}))
        harness=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Harness')
        keywords={k.arg:k.value for k in harness.keywords}
        self.assertIs(keywords['record_video'].value,False)
        self.assertNotIn('frame_callback',keywords)

    def test_safe_cusp_pose_recovery_is_bounded_by_original_junctions(self):
        recovery=dict(kind='plan_status',status='replanning_after_maneuver',reason='actual_cusp_pose_differs')
        self.assertIsNone(execution_issue([recovery],2))
        self.assertEqual(execution_issue([recovery]*3,2),'excessive_handoff_recovery')
        self.assertEqual(execution_issue([dict(recovery,reason='footprint_collision')],2),'unexpected_handoff_recovery')

    def test_recovery_does_not_hide_follow_failure(self):
        self.assertEqual(execution_issue([dict(kind='plan_status',status='follow_failed')],2),'execution_failure:follow_failed')

    def test_motion_requires_actual_gear_speed_and_fresh_acknowledged_telemetry(self):
        def row(gear,**kw):return dict(telemetry_ok=True,control_acknowledged=True,speed=.5,actual=dict(gear_index=gear),**kw)
        good=row(1);reverse=row(-1)
        self.assertEqual(observed_motion_directions([good,reverse]),[-1,1])
        bad=[row(None),row(float('nan')),row(True),dict(good,telemetry_ok=False),
             dict(reverse,control_acknowledged=False),dict(reverse,speed=0.)]
        self.assertEqual(observed_motion_directions(bad),[])


if __name__=='__main__':unittest.main()
