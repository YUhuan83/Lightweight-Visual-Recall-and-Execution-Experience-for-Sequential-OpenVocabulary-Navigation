#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include "graphnav_planner/planner.hpp"

namespace {
using Graph = graphnav_msgs::msg::NavigationGraph;
geometry_msgs::msg::Point point(double x, double y=0., double z=0.) {
  geometry_msgs::msg::Point p; p.x=x; p.y=y; p.z=z; return p;
}
std::shared_ptr<Graph> fixture(std::vector<geometry_msgs::msg::Point> points) {
  auto g=std::make_shared<Graph>(); g->header.frame_id="spot1/odom";
  g->header.stamp.sec=100; g->trav_classes={"default"}; g->current_node_idx=0;
  for (double x : {0.,2.}) {
    graphnav_msgs::msg::Node n; n.pose.position.x=x; n.pose.orientation.w=1.;
    n.trav_properties.resize(1); g->nodes.push_back(n);
  }
  g->nodes[1].trav_properties[0].is_frontier=true;
  g->nodes[1].trav_properties[0].frontier_points=points;
  graphnav_msgs::msg::Edge e; e.from_idx=0; e.to_idx=1;
  e.traversability.resize(1); e.traversability[0].traversability_cost=2.;
  g->edges.push_back(e); return g;
}
}

TEST(FrontierEndpoint, ScoredForwardExitMustNotBecomeBackwardCentroid) {
  auto g=fixture({point(3.),point(-8.)});
  graphnav_planner::Planner planner(rclcpp::get_logger("endpoint"));
  planner.set_trav_class("default"); planner.update_graph(g);
  Eigen::Vector3d goal(50.,0.,0.);
  auto path=planner.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
  ASSERT_FALSE(path.empty()); EXPECT_DOUBLE_EQ(path.back().x(),3.);
  EXPECT_FALSE(planner.route_complete());
}

TEST(FrontierEndpoint, NoInventedPointBetweenDisjointExits) {
  auto g=fixture({point(3.,4.),point(3.,-4.)});
  graphnav_planner::Planner planner(rclcpp::get_logger("endpoint"));
  planner.set_trav_class("default"); planner.update_graph(g);
  Eigen::Vector3d goal(50.,4.,0.);
  auto path=planner.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
  ASSERT_FALSE(path.empty()); EXPECT_DOUBLE_EQ(path.back().x(),3.);
  EXPECT_DOUBLE_EQ(path.back().y(),4.); EXPECT_FALSE(planner.route_complete());
}

TEST(FrontierEndpoint, ReplanUsesCurrentDestinationAndPreservesPartialStatus) {
  auto g=fixture({point(3.),point(-8.)});
  graphnav_planner::Planner planner(rclcpp::get_logger("endpoint"));
  planner.set_trav_class("default"); planner.update_graph(g);
  Eigen::Vector3d forward(50.,0.,0.),backward(-50.,0.,0.);
  auto first=planner.plan_to_goal(forward,.5,rclcpp::Time(100,0,RCL_ROS_TIME));
  auto second=planner.plan_to_goal(backward,.5,rclcpp::Time(101,0,RCL_ROS_TIME));
  ASSERT_FALSE(first.empty()); ASSERT_FALSE(second.empty());
  EXPECT_DOUBLE_EQ(first.back().x(),3.); EXPECT_DOUBLE_EQ(second.back().x(),-8.);
  EXPECT_FALSE(planner.route_complete());
}

TEST(FrontierEndpoint, SingleExitAndInputPermutationHaveSameEndpoint) {
  for(auto points : {std::vector{point(3.)},std::vector{point(3.),point(-8.)},std::vector{point(-8.),point(3.)}}) {
    auto g=fixture(points); graphnav_planner::Planner planner(rclcpp::get_logger("endpoint"));
    planner.set_trav_class("default");planner.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    auto path=planner.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
    ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),3.);
  }
}

TEST(FrontierEndpoint, InvalidHeightCannotSupplyScoreForAnotherExit) {
  auto g=fixture({point(45.,0.,std::numeric_limits<double>::quiet_NaN()),point(3.)});
  graphnav_planner::Planner planner(rclcpp::get_logger("endpoint"));
  planner.set_trav_class("default");planner.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
  auto path=planner.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
  ASSERT_FALSE(path.empty()); EXPECT_DOUBLE_EQ(path.back().x(),3.);
  EXPECT_FALSE(planner.route_complete());
}

TEST(FrontierEndpoint, InvalidOnlyFrontierCannotBecomeExecutableRoute) {
  auto g=fixture({point(45.,0.,std::numeric_limits<double>::quiet_NaN())});
  graphnav_planner::Planner planner(rclcpp::get_logger("endpoint"));
  planner.set_trav_class("default");planner.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
  auto path=planner.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
  EXPECT_TRUE(path.empty()); EXPECT_FALSE(planner.route_complete());
}


TEST(HeadingFreshness, OldBlockedHeadingDoesNotOutliveItsPhysicalAssessment) {
  for (int stamp : {90,94,95,100}) {
    auto g=fixture({point(3.)});
    graphnav_msgs::msg::RobotNodeTraversability a;
    a.robot_profile_id="default";a.observed_at.sec=stamp;a.confidence=.8;
    a.feasible=false;a.physical_traversability=0.;
    g->nodes[1].robot_traversability.push_back(a);
    graphnav_msgs::msg::HeadingCandidate heading;
    heading.traversable=false;heading.heading=0.;heading.predicted_utility=.8;
    g->nodes[1].heading_candidates.push_back(heading);
    graphnav_planner::Planner p(rclcpp::get_logger("heading_freshness"));
    p.set_trav_class("default");p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
    if(stamp<95) { ASSERT_FALSE(path.empty())<<stamp;EXPECT_DOUBLE_EQ(path.back().x(),3.); }
    else EXPECT_TRUE(path.empty())<<stamp;
  }
}

TEST(HeadingFreshness, FreshAllowedHeadingAndFreshLowConfidenceDenialKeepTheirMeaning) {
  for(bool allowed : {false,true}) {
    auto g=fixture({point(3.)});
    graphnav_msgs::msg::RobotNodeTraversability a;
    a.robot_profile_id="default";a.observed_at=g->header.stamp;
    a.confidence=allowed?.8:0.;a.feasible=allowed;a.physical_traversability=allowed?1.:0.;
    g->nodes[1].robot_traversability.push_back(a);
    graphnav_msgs::msg::HeadingCandidate h;
    h.traversable=allowed;h.heading=0.;h.predicted_utility=.8;
    g->nodes[1].heading_candidates.push_back(h);
    graphnav_planner::Planner p(rclcpp::get_logger("heading_freshness"));
    p.set_trav_class("default");p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    EXPECT_EQ(!p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp)).empty(),allowed);
  }
}


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
  auto g=clearance_fixture();graphnav_planner::Planner p(rclcpp::get_logger("clearance"));p.set_trav_class("default");
  p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);auto now=rclcpp::Time(g->header.stamp);
  auto strict=p.plan_to_goal(goal,.5,now);ASSERT_FALSE(strict.empty());EXPECT_DOUBLE_EQ(strict.back().x(),-3.);
  auto forward=p.plan_to_goal(goal,.5,now,g);ASSERT_FALSE(forward.empty());EXPECT_DOUBLE_EQ(forward.back().x(),3.);
  EXPECT_EQ(g->edges[0].robot_traversability[0].physical_traversability,0.);
  auto again=p.plan_to_goal(goal,.5,now);ASSERT_FALSE(again.empty());EXPECT_DOUBLE_EQ(again.back().x(),-3.);
}

TEST(ExplicitClearanceGuidance, NarrowObstacleOrTerrainViolationRemainsBlocked){
  for(auto values:{std::pair(.49f,0.f),std::pair(.8f,.5f)}){
    auto g=clearance_fixture(values.first,values.second);graphnav_planner::Planner p(rclcpp::get_logger("clearance"));
    p.set_trav_class("default");p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp),g);
    ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),-3.);
  }
}

TEST(ExplicitClearanceGuidance, MissingBaseEdgeCannotBeInvented){
  auto g=clearance_fixture();g->edges.erase(g->edges.begin());
  graphnav_planner::Planner p(rclcpp::get_logger("clearance"));p.set_trav_class("default");p.update_graph(g);
  Eigen::Vector3d goal(50.,0.,0.);auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp),g);
  ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),-3.);
}

TEST(ExplicitClearanceGuidance, ActiveTraversalClassIsPreserved){
  auto g=clearance_fixture();g->trav_classes={"road"};
  graphnav_planner::Planner p(rclcpp::get_logger("clearance"));p.set_trav_class("road");p.update_graph(g);
  Eigen::Vector3d goal(50.,0.,0.);auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp),g);
  ASSERT_FALSE(path.empty());EXPECT_DOUBLE_EQ(path.back().x(),3.);
}
