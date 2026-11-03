#include <gtest/gtest.h>
#include "graphnav_planner/planner.hpp"
#include "graphnav_planner/exploration_utility.hpp"
#include "graphnav_planner/exploration_backoff.hpp"
static std::shared_ptr<graphnav_msgs::msg::NavigationGraph> connected_graph() {
  auto graph=std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  graph->header.frame_id="spot1/odom";graph->header.stamp.sec=10;
  graph->trav_classes={"default"};graph->nodes.resize(3);graph->current_node_idx=0;
  for(size_t i=0;i<3;++i) {
    auto & n=graph->nodes[i];n.pose.position.x=5.*i;n.pose.position.z=52.;n.pose.orientation.w=1.;
    n.trav_properties.resize(1);n.trav_properties[0].free_radius=5.;n.trav_properties[0].explored_radius=5.;
  }
  for(size_t i=1;i<3;++i) {
    graphnav_msgs::msg::Edge edge;edge.from_idx=i-1;edge.to_idx=i;
    edge.traversability.resize(1);edge.traversability[0].traversability_cost=5.;graph->edges.push_back(edge);
  }
  return graph;
}
TEST(GraphPlanning, KnownGoalUsesConnectedMeasuredGraph) {
  graphnav_planner::Planner planner(rclcpp::get_logger("core_test"));
  planner.set_trav_class("default");
  planner.update_graph(connected_graph());
  Eigen::Vector3d goal(10.,0.,52.);
  auto path=planner.plan_to_goal(goal,3.,rclcpp::Time(10,0));
  ASSERT_FALSE(path.empty());EXPECT_TRUE(planner.route_complete());
  EXPECT_NEAR((path.back()-goal).norm(),0.,1e-5);
}
TEST(GraphPlanning, ElevatedObjectUsesPlanarConnectionAndRetainsMeasuredHeight) {
  graphnav_planner::Planner planner(rclcpp::get_logger("core_test"));
  planner.set_trav_class("default");
  planner.update_graph(connected_graph());
  Eigen::Vector3d goal(12.99,0.,70.);
  auto path=planner.plan_to_goal(goal,3.,rclcpp::Time(10,0));
  ASSERT_FALSE(path.empty());EXPECT_TRUE(planner.route_complete());
  EXPECT_DOUBLE_EQ(path.back().z(),70.);
  EXPECT_NEAR((path.back()-goal).norm(),0.,1e-5);
}
TEST(GraphPlanning, PlanarConnectionKeepsStrictThreeMeterBoundary) {
  graphnav_planner::Planner planner(rclcpp::get_logger("core_test"));
  planner.set_trav_class("default");
  planner.update_graph(connected_graph());
  Eigen::Vector3d goal(13.,0.,70.);
  auto path=planner.plan_to_goal(goal,3.,rclcpp::Time(10,0));
  EXPECT_FALSE(planner.route_complete());EXPECT_TRUE(path.empty());
}
TEST(GraphPlanning, FrontierReselectionNeedsOneVisitPenaltyOfImprovement) {
  EXPECT_FALSE(graphnav_planner::frontier_candidate_should_preempt(.22418,.21373));
  EXPECT_TRUE(graphnav_planner::frontier_candidate_should_preempt(.88928,.22466));
}
TEST(GraphPlanning, FrontierNodeSurvivalDoesNotKeepARemovedExit) {
  geometry_msgs::msg::Point chosen,exit;
  chosen.x=7.;exit.x=7.2;
  EXPECT_TRUE(graphnav_planner::frontier_endpoint_current(chosen,{exit}));
  exit.x=5.;exit.y=2.;
  EXPECT_FALSE(graphnav_planner::frontier_endpoint_current(chosen,{exit}));
  EXPECT_FALSE(graphnav_planner::frontier_endpoint_current(chosen,{}));
}
TEST(GraphPlanning, FailedRoutePostponesOnlyItsDestinationFromTheSameStart) {
  graphnav_planner::ExplorationBackoff feedback;
  geometry_msgs::msg::PoseStamped refusal;
  refusal.header.frame_id="spot1/odom";refusal.header.stamp.sec=10;
  refusal.pose.position.x=8.;
  geometry_msgs::msg::Point destination,start,other;
  destination.x=20.;other.x=21.;
  feedback.remember(refusal.header,refusal.pose.position,destination,"frontier-a");
  const int64_t now=11000000000LL;
  EXPECT_FALSE(feedback.reject(refusal,now,false,start));
  auto unrelated=refusal;unrelated.pose.position.x=7.;
  EXPECT_FALSE(feedback.reject(unrelated,now,true,start));
  ASSERT_TRUE(feedback.reject(refusal,now,true,start));
  EXPECT_TRUE(feedback.postponed(destination,now,start));
  EXPECT_FALSE(feedback.postponed(other,now,start));
  start.x=1.;EXPECT_FALSE(feedback.postponed(destination,now,start));
  start.x=0.;
  EXPECT_FALSE(feedback.postponed(destination,now+graphnav_planner::rejected_frontier_backoff_ns,start));
  feedback.clear();EXPECT_FALSE(feedback.postponed(destination,now,start));
}

TEST(GraphPlanning, SelectedFrontierKeepsItsMeasuredExitBeyondTheGoalRadius) {
  auto graph=connected_graph();
  auto& owner=graph->nodes[2];owner.uuid.id[0]=3;
  owner.trav_properties[0].is_frontier=true;
  geometry_msgs::msg::Point exit;exit.x=14.;exit.z=52.;
  owner.trav_properties[0].frontier_points={exit};
  graphnav_planner::Planner planner(rclcpp::get_logger("frontier_test"));
  planner.set_trav_class("default");planner.update_graph(graph);
  auto route=planner.plan_to_frontier(graphnav_planner::uuid_to_string(owner.uuid),exit,rclcpp::Time(10,0));
  ASSERT_EQ(route.size(),4u);EXPECT_TRUE(planner.route_complete());
  EXPECT_DOUBLE_EQ(route[2].x(),10.);EXPECT_DOUBLE_EQ(route.back().x(),14.);
  exit.y=1.;
  EXPECT_TRUE(planner.plan_to_frontier(graphnav_planner::uuid_to_string(owner.uuid),exit,rclcpp::Time(10,0)).empty());
}

TEST(GraphPlanning, DisconnectedFrontierDoesNotPassByRoutingToAnotherFrontier) {
  auto graph=connected_graph();graph->edges.pop_back();
  for(size_t i=1;i<3;++i) {
    auto& n=graph->nodes[i];n.uuid.id[0]=i+1;
    n.trav_properties[0].is_frontier=true;
    geometry_msgs::msg::Point exit;exit.x=5.*i+4.;exit.z=52.;
    n.trav_properties[0].frontier_points={exit};
  }
  graphnav_planner::Planner planner(rclcpp::get_logger("frontier_test"));
  planner.set_trav_class("default");planner.update_graph(graph);
  const auto& blocked=graph->nodes[2];const auto& alternate=graph->nodes[1];
  EXPECT_TRUE(planner.plan_to_frontier(graphnav_planner::uuid_to_string(blocked.uuid),
      blocked.trav_properties[0].frontier_points[0],rclcpp::Time(10,0)).empty());
  EXPECT_FALSE(planner.route_complete());
  const auto route=planner.plan_to_frontier(graphnav_planner::uuid_to_string(alternate.uuid),
      alternate.trav_properties[0].frontier_points[0],rclcpp::Time(10,0));
  ASSERT_FALSE(route.empty());EXPECT_DOUBLE_EQ(route.back().x(),9.);
}
