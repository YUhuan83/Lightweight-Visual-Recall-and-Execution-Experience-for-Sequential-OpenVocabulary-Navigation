from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v74_document_handoff/candidate/nav2_adapter')
p=root/'wildos_nav2_adapter/graph_path_bridge.py';s=p.read_text()
old="        if ((reason == 'empty_path' or reason.startswith('local_ahead_')) and self.active_plan_start is not None"
new="        if (reason == 'empty_path' and self.active_plan_start is not None"
assert old in s;s=s.replace(old,new,1);p.write_text(s)
p=root/'wildos_nav2_adapter/continuous_graph_path_bridge.py';s=p.read_text()
old='''        if (not self.upstream_target_approach or self.object_destination is None
                or self.expected is None or self.memory_goal is not None or self.memory_selecting):'''
new='''        if (self.carrier_model!='ackermann' or not self.upstream_target_approach or self.object_destination is None
                or self.expected is None or self.memory_goal is not None or self.memory_selecting):'''
assert old in s;s=s.replace(old,new,1);p.write_text(s)
p=root/'test/test_continuous_exploration_ros.py';s=p.read_text();where=s.index('    def test_missing_execution_progress_does_not_start_obstacle_replan')
s=s[:where]+'''    def test_local_ahead_rejection_is_not_cached_as_a_global_planner_failure(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((20.,0.),'frontier-A');n.route_endpoint=(20.,0.)
        n.candidates=[(20.,0.,0.,20.)];n.candidate_index=0;n.active_plan_start=(0.,0.)
        n.active_plan_failure_context=n.failed_goal_context()
        n.vehicle_pose_feasible=Mock(return_value=True);n.try_candidate=Mock()
        n.candidate_failed('local_ahead_local_collision_cell')
        # A new local observation can clear while the global map is unchanged.
        self.assertFalse(n.goal_recently_failed((0.,0.),(20.,0.),0.))
        n.carrier_model='ideal_point'
        self.assertEqual(n.target_forward_candidates((0.,0.)),[])

''' +s[where:];p.write_text(s)
print('Local rejections retain local-map scope; ideal carrier retains its original planning flow')
