from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan/candidate')
p=root/'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner.cpp';s=p.read_text()
s=s.replace('const std::string& profile_id, rclcpp::Time now, double ttl,',
            'const std::string& profile_id, const std::string& trav_class, rclcpp::Time now, double ttl,')
s=s.replace('source->trav_classes.end(),"default")','source->trav_classes.end(),trav_class)')
s=s.replace('clearance_guidance_graph(explicit_topology,robot_profile_id_,current_time,',
            'clearance_guidance_graph(explicit_topology,robot_profile_id_,trav_class_,current_time,')
p.write_text(s)
p=root/'nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py';s=p.read_text()
old='''        if self.candidates and self.odom is not None and self.expected is not None:
            start=xy(self.odom.pose.pose);first=self.candidates[0]
            away=((first[0]-start[0])*(self.expected[0]-start[0])
                  +(first[1]-start[1])*(self.expected[1]-start[1])<=0)'''
new='''        if self.odom is not None and self.expected is not None:
            start=xy(self.odom.pose.pose)
            away=(not self.candidates or
                  (self.candidates[0][0]-start[0])*(self.expected[0]-start[0])
                  +(self.candidates[0][1]-start[1])*(self.expected[1]-start[1])<=0)'''
assert old in s;s=s.replace(old,new,1);p.write_text(s)
p=root/'nav2_adapter/test/test_continuous_exploration_ros.py';s=p.read_text();where=s.index('    def test_missing_execution_progress_does_not_start_obstacle_replan')
s=s[:where]+'''    def test_explicit_target_without_graph_candidates_still_probes_forward(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((20.,0.),'frontier-A');n.route_endpoint=(20.,0.)
        n.candidates=[];n.vehicle_pose_feasible=Mock(return_value=True)
        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][:2],(8.,0.))

''' +s[where:];p.write_text(s)
p=root/'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/test/test_frontier_endpoint.cpp'
with p.open('a') as f:f.write('''
TEST(ExplicitClearanceGuidance, ActiveTraversalClassIsPreserved){
  auto g=clearance_fixture();g->trav_classes={"road"};
  graphnav_planner::Planner p(rclcpp::get_logger("clearance"));p.set_trav_class("road");p.update_graph(g);
  Eigen::Vector3d goal(50.,0.,0.);auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp),g);
  ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),3.);
}
''')
print('Final regression cases: empty graph candidates and non-default traversal class')
