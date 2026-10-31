"""User-approved step 3: conditional explicit-goal clearance guidance, not execution permission."""
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
package=root/'candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner'
p=package/'include/graphnav_planner/planner.hpp';s=p.read_text()
old='plan_to_goal(Eigen::Vector3d& goal, double goal_radius, rclcpp::Time current_time);'
new='plan_to_goal(Eigen::Vector3d& goal, double goal_radius, rclcpp::Time current_time, graphnav_msgs::msg::NavigationGraph::ConstSharedPtr explicit_topology = nullptr);'
assert old in s;s=s.replace(old,new);p.write_text(s)
p=package/'src/planner.cpp';s=p.read_text()
where=s.index('\n}\n\nPlanner::Planner')
s=s[:where]+'''

bool within_terrain_limit(double value, double limit)
{
  return std::isfinite(value) && value >= 0. && (limit < 0. || value <= limit);
}

graphnav_msgs::msg::NavigationGraph::SharedPtr clearance_guidance_graph(
    graphnav_msgs::msg::NavigationGraph::ConstSharedPtr source,
    const std::string& profile_id, rclcpp::Time now, double ttl,
    const Eigen::Vector3d& goal, double penalty)
{
  const auto profile=std::find_if(source->robot_profiles.begin(),source->robot_profiles.end(),
      [&](const auto& p){return p.profile_id==profile_id;});
  if(profile==source->robot_profiles.end() || source->current_node_idx>=source->nodes.size())return nullptr;
  double minimum=0.;
  for(const auto&p:profile->properties)
    if(p.key=="minimum_horizontal_clearance" && p.value.size()==1)minimum=p.value[0];
  if(!std::isfinite(minimum) || minimum<=.5)return nullptr;
  const auto eligible=[&](const auto&a){
    return assessment_fresh(&a,now,ttl) && std::isfinite(a.minimum_clearance) &&
        a.minimum_clearance>=.5 && a.minimum_clearance<minimum &&
        std::isfinite(a.dynamic_risk) && a.dynamic_risk>=0. && a.dynamic_risk<1. &&
        within_terrain_limit(a.maximum_step_height,profile->max_step_height) &&
        within_terrain_limit(a.roughness,profile->max_roughness);
  };
  auto guide=std::make_shared<graphnav_msgs::msg::NavigationGraph>(*source);
  bool changed=false;
  for(auto&n:guide->nodes)for(auto&a:n.robot_traversability)
    if(a.robot_profile_id==profile_id && !a.feasible && eligible(a) &&
        within_terrain_limit(a.maximum_uphill_slope,profile->max_uphill_slope) &&
        within_terrain_limit(a.maximum_downhill_slope,profile->max_downhill_slope) &&
        within_terrain_limit(a.maximum_cross_slope,profile->max_cross_slope)){
      // Planning-only copy: raw measurements and recall evidence stay intact.
      a.feasible=true;a.physical_traversability=a.minimum_clearance/minimum;
      n.heading_candidates.clear();changed=true;
    }
  const auto trav=std::find(source->trav_classes.begin(),source->trav_classes.end(),"default");
  if(trav==source->trav_classes.end())return nullptr;
  const size_t index=std::distance(source->trav_classes.begin(),trav);
  for(auto&e:guide->edges)for(auto&a:e.robot_traversability)
    if(index<e.traversability.size() && std::isfinite(e.traversability[index].traversability_cost) &&
        e.traversability[index].traversability_cost>=0. && a.robot_profile_id==profile_id &&
        a.physical_traversability<=0. && eligible(a) &&
        within_terrain_limit(a.maximum_slope,profile->max_uphill_slope) &&
        within_terrain_limit(a.maximum_slope,profile->max_downhill_slope) &&
        within_terrain_limit(a.maximum_slope,profile->max_cross_slope)){
      a.physical_traversability=a.minimum_clearance/minimum;
      e.traversability[index].traversability_cost+=penalty*minimum/a.minimum_clearance;
      changed=true;
    }
  if(!changed)return nullptr;
  const auto&origin=source->nodes[source->current_node_idx].pose.position;
  const double initial=std::hypot(goal.x()-origin.x,goal.y()-origin.y);
  // This pass exists only because the strict graph has no forward exit.
  // Keep measured forward exits; never invent a connection or endpoint.
  for(auto&n:guide->nodes)if(index<n.trav_properties.size()){
    auto&exit=n.trav_properties[index];
    std::erase_if(exit.frontier_points,[&](const auto&p){
      return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
          std::hypot(goal.x()-p.x,goal.y()-p.y)>=initial;
    });
    if(exit.frontier_points.empty())exit.is_frontier=false;
  }
  return guide;
}
''' +s[where:]
old='Planner::plan_to_goal(Eigen::Vector3d& goal, double goal_radius, rclcpp::Time current_time)'
new='Planner::plan_to_goal(Eigen::Vector3d& goal, double goal_radius, rclcpp::Time current_time, graphnav_msgs::msg::NavigationGraph::ConstSharedPtr explicit_topology)'
assert old in s;s=s.replace(old,new)
old='''  graph_.remove_vertex(virtual_goal);
  return path_points;'''
new='''  const auto& origin=graph_.get_vertex(current_node_idx_).pose.position;
  const double initial_gap=std::hypot(goal.x()-origin.x,goal.y()-origin.y);
  bool forward_available=false;
  if(explicit_topology)for(const auto&[id,score]:frontier_scores_){
    const auto&end=frontier_endpoints.at(id);
    if(std::hypot(goal.x()-end.x(),goal.y()-end.y())<initial_gap &&
        graaf::algorithm::dijkstra_shortest_path(graph_,current_node_idx_,id)){
      forward_available=true;break;
    }
  }
  graph_.remove_vertex(virtual_goal);
  if(explicit_topology && !forward_available){
    auto guide=clearance_guidance_graph(explicit_topology,robot_profile_id_,current_time,
        stale_observation_seconds_,goal,physical_risk_factor_);
    if(guide){
      auto strict_graph=std::move(graph_);auto strict_map=std::move(unexplored_space_map_);
      auto strict_scores=std::move(frontier_scores_);
      const auto old_frontier=latest_frontier_;const auto old_time=latest_frontier_time_;
      update_graph(guide);
      auto forward=plan_to_goal(goal,goal_radius,current_time);
      graph_=std::move(strict_graph);unexplored_space_map_=std::move(strict_map);
      if(!forward.empty() && std::hypot(goal.x()-forward.back().x(),goal.y()-forward.back().y())<initial_gap)
        return forward;
      frontier_scores_=std::move(strict_scores);latest_frontier_=old_frontier;latest_frontier_time_=old_time;
      route_complete_=false;
    }
  }
  return path_points;'''
assert s.count(old)==1;s=s.replace(old,new);p.write_text(s)
p=package/'src/planner_node.cpp';s=p.read_text()
old='planner_.plan_to_goal(goal_vec, goal_radius_, this->get_clock()->now());'
new='planner_.plan_to_goal(goal_vec, goal_radius_, this->get_clock()->now(), exploring ? nullptr : latest_graph_);'
assert old in s;s=s.replace(old,new);p.write_text(s)
p=package/'test/test_frontier_endpoint.cpp';s=p.read_text()
s+='''

namespace {
std::shared_ptr<Graph> clearance_fixture(float clearance=.8f,float slope=0.f){
  auto g=fixture({point(3.)});
  auto rear=g->nodes[1];rear.pose.position.x=-2.;rear.trav_properties[0].frontier_points={point(-3.)};
  g->nodes.push_back(rear);auto rear_edge=g->edges[0];rear_edge.to_idx=2;g->edges.push_back(rear_edge);
  graphnav_msgs::msg::RobotMotionConstraints profile;profile.profile_id="default";
  profile.max_uphill_slope=profile.max_downhill_slope=profile.max_cross_slope=.3;
  profile.max_step_height=.2;profile.max_roughness=.2;
  graphnav_msgs::msg::KeyValue minimum;minimum.key="minimum_horizontal_clearance";minimum.value={1.06};
  profile.properties.push_back(minimum);g->robot_profiles.push_back(profile);
  graphnav_msgs::msg::RobotEdgeTraversability a;a.robot_profile_id="default";
  a.observed_at=g->header.stamp;a.confidence=.8;a.physical_traversability=0.;
  a.minimum_clearance=clearance;a.maximum_slope=slope;
  g->edges[0].robot_traversability.push_back(a);
  graphnav_msgs::msg::KeyValue violation;violation.key="physical_violation_observed";violation.value={1.};
  g->edges[0].properties.push_back(violation);return g;
}
}

TEST(ExplicitClearanceGuidance, ForwardFiniteTopologyIsGuidanceOnlyForExplicitGoal){
  auto g=clearance_fixture();Planner p(rclcpp::get_logger("clearance"));p.set_trav_class("default");
  p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);auto now=rclcpp::Time(g->header.stamp);
  auto strict=p.plan_to_goal(goal,.5,now);ASSERT_FALSE(strict.empty());EXPECT_DOUBLE_EQ(strict.back().x(),-3.);
  auto forward=p.plan_to_goal(goal,.5,now,g);ASSERT_FALSE(forward.empty());EXPECT_DOUBLE_EQ(forward.back().x(),3.);
  EXPECT_EQ(g->edges[0].robot_traversability[0].physical_traversability,0.);
  auto again=p.plan_to_goal(goal,.5,now);ASSERT_FALSE(again.empty());EXPECT_DOUBLE_EQ(again.back().x(),-3.);
}

TEST(ExplicitClearanceGuidance, NarrowObstacleOrTerrainViolationRemainsBlocked){
  for(auto values:{std::pair(.49f,0.f),std::pair(.8f,.5f)}){
    auto g=clearance_fixture(values.first,values.second);Planner p(rclcpp::get_logger("clearance"));
    p.set_trav_class("default");p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp),g);
    ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),-3.);
  }
}

TEST(ExplicitClearanceGuidance, MissingBaseEdgeCannotBeInvented){
  auto g=clearance_fixture();g->edges.erase(g->edges.begin());
  Planner p(rclcpp::get_logger("clearance"));p.set_trav_class("default");p.update_graph(g);
  Eigen::Vector3d goal(50.,0.,0.);auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp),g);
  ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),-3.);
}
'''
s=s.replace('Planner p(rclcpp::get_logger("clearance"))','graphnav_planner::Planner p(rclcpp::get_logger("clearance"))')
p.write_text(s)
print('Step3 applied: explicit-only, finite topology, clearance-only fallback; no new components')
