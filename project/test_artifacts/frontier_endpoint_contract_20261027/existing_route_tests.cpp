#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <limits>
#include "graphnav_planner/planner.hpp"
#include "graphnav_planner/exploration_backoff.hpp"
#include "graphnav_planner/exploration_observation.hpp"
#include "graphnav_planner/exploration_target.hpp"
#include "graphnav_planner/exploration_utility.hpp"
#include "graphnav_planner/graph_score_overlay.hpp"
#include "graphnav_planner/destination_fence.hpp"

using graphnav_planner::Planner;
using Graph = graphnav_msgs::msg::NavigationGraph;

namespace
{
std::shared_ptr<Graph> graph(std::initializer_list<double> xs)
{
  auto g = std::make_shared<Graph>();
  g->header.frame_id = "spot1/odom";
  g->header.stamp.sec = 100;
  g->trav_classes = {"default"};
  g->current_node_idx = 0;
  for (double x : xs)
  {
    graphnav_msgs::msg::Node n;
    n.pose.position.x = x;
    n.pose.orientation.w = 1.0;
    n.trav_properties.resize(1);
    graphnav_msgs::msg::RobotNodeTraversability a;
    a.robot_profile_id = "default";
    a.feasible = true;
    a.physical_traversability = 1.0f;
    a.confidence = 0.8f;
    a.observed_at.sec = 100;
    n.robot_traversability.push_back(a);
    g->nodes.push_back(n);
  }
  return g;
}
void edge(Graph& g, size_t from, size_t to, double cost,
          bool blocked = false, bool traversed = false, int observed = 100, float confidence = .8f)
{
  graphnav_msgs::msg::Edge e;
  e.from_idx = from;
  e.to_idx = to;
  e.traversability.resize(1);
  e.traversability[0].traversability_cost = cost;
  graphnav_msgs::msg::KeyValue evidence;
  evidence.key = "traversed";
  evidence.value = {traversed ? 1.0f : 0.0f};
  e.traversability[0].properties.push_back(evidence);
  graphnav_msgs::msg::RobotEdgeTraversability a;
  a.robot_profile_id = "default";
  a.physical_traversability = blocked ? 0.f : 1.f;
  a.confidence = confidence;
  a.observed_at.sec = observed;
  e.robot_traversability.push_back(a);
  g.edges.push_back(e);
}
Planner planner()
{
  Planner p(rclcpp::get_logger("offline_planner_contract"));
  p.set_trav_class("default");
  return p;
}
void frontier(Graph& g, size_t i)
{
  auto& n = g.nodes[i];
  n.trav_properties[0].is_frontier = true;
  geometry_msgs::msg::Point point;
  point.x = n.pose.position.x + 1.;
  n.trav_properties[0].frontier_points.push_back(point);
}
}

TEST(RouteMechanism, KnownRouteIsPreferredOverFrontierHeuristic)
{
  auto g = graph({0., 5., 10., 2.});
  edge(*g, 0, 1, 5.);
  edge(*g, 1, 2, 5.);
  edge(*g, 0, 3, .1);
  frontier(*g, 3);
  auto p = planner();
  p.update_graph(g);
  Eigen::Vector3d goal(10., 0., 0.);
  const auto path = p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp));
  ASSERT_TRUE(p.route_complete());
  ASSERT_GE(path.size(), 3u);
  EXPECT_DOUBLE_EQ(path[1].x(), 5.);
  EXPECT_DOUBLE_EQ(path.back().x(), 10.);
  // A known route clears the latest frontier but can still be visualized.
  EXPECT_FALSE(p.get_score_visualization(rclcpp::Time(g->header.stamp), "spot1/odom", true).markers.empty());
}

TEST(RouteMechanism, FrontierPrefixDoesNotContainUnseenFiftyMetreTail)
{
  auto g = graph({0., 2.});
  edge(*g, 0, 1, 2.);
  frontier(*g, 1);
  auto p = planner();
  p.update_graph(g);
  Eigen::Vector3d goal(50., 0., 0.);
  const auto path = p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp));
  ASSERT_FALSE(p.route_complete());
  ASSERT_FALSE(path.empty());
  EXPECT_LT(path.back().x(), 5.);
  // Replanning now takes the local-frontier continuity branch, whose terminal
  // attachment must have the same direction as the initial global choice.
  const auto continued=p.plan_to_goal(goal,.5,rclcpp::Time(101,0,RCL_ROS_TIME));
  ASSERT_FALSE(continued.empty());
  EXPECT_LT(continued.back().x(),5.);
}

TEST(RouteMechanism, SparseFrontierScoresDoNotDependOnNodeOrdering)
{
  for (bool reverse : {false, true}) {
    auto g = reverse ? graph({0., 10., 20.}) : graph({0., 20., 10.});
    const size_t scored = reverse ? 1 : 2;
    const size_t unscored = reverse ? 2 : 1;
    for (size_t i=0; i<g->nodes.size(); ++i)
      g->nodes[i].uuid.id[0] = static_cast<uint8_t>(i+1);
    frontier(*g, scored); frontier(*g, unscored);
    edge(*g, 0, scored, 1.); edge(*g, 0, unscored, 1.);
    graphnav_msgs::msg::KeyValue scores;
    scores.key = "frontier_scores";
    scores.value = std::vector<float>(16, 1.f);
    g->nodes[scored].properties.push_back(scores);
    auto p = planner(); p.update_graph(g);
    Eigen::Vector3d goal(100., 0., 0.);
    const auto path = p.plan_to_goal(goal, .5,
                                     rclcpp::Time(g->header.stamp));
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.back().x(), 11., 1e-6);
    EXPECT_FALSE(p.route_complete());
  }
}

TEST(RouteMechanism, FreshObservedBlockRemovesUntraversedConnection)
{
  auto g = graph({0., 10.});
  edge(*g, 0, 1, 10., true);
  auto p = planner(); p.update_graph(g);
  Eigen::Vector3d goal(10., 0., 0.);
  EXPECT_TRUE(p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp)).empty());
}

TEST(RouteMechanism, DrivenConnectionIsRetainedAsPenalizedTopologyPrior)
{
  auto g = graph({0., 10.});
  edge(*g, 0, 1, 10., true, true);
  auto p = planner(); p.update_graph(g);
  Eigen::Vector3d goal(10., 0., 0.);
  EXPECT_FALSE(p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp)).empty());
  EXPECT_TRUE(p.route_complete()); // Does not establish local vehicle-path clearance.
}

TEST(RouteMechanism, FreshMeasuredViolationOverridesDrivenTopologyPrior)
{
  auto g=graph({0.,10.});edge(*g,0,1,10.,true,true);
  graphnav_msgs::msg::KeyValue witness;
  witness.key="physical_violation_observed";witness.value={1.f};
  g->edges[0].properties.push_back(witness);
  auto p=planner();p.update_graph(g);Eigen::Vector3d goal(10.,0.,0.);
  EXPECT_TRUE(p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp)).empty());
  g->edges[0].properties[0].value={0.f};
  g->edges[0].robot_traversability[0].physical_traversability=1.f;
  g->header.stamp.sec=101;g->edges[0].robot_traversability[0].observed_at.sec=101;
  p.update_graph(g);
  EXPECT_FALSE(p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp)).empty());
}

TEST(RouteMechanism, FreshMeasuredNodeViolationCannotBeBypassedByVisitedFlag)
{
  auto g=graph({0.,10.});edge(*g,0,1,10.,false,true);
  graphnav_msgs::msg::KeyValue witness;
  witness.key="physical_violation_observed";witness.value={1.f};
  g->nodes[1].properties.push_back(witness);
  g->nodes[1].robot_traversability[0].feasible=false;
  graphnav_msgs::msg::KeyValue visited;visited.key="visited";visited.value={1.f};
  g->nodes[1].trav_properties[0].properties.push_back(visited);
  auto p=planner();p.update_graph(g);Eigen::Vector3d goal(10.,0.,0.);
  EXPECT_TRUE(p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp)).empty());
}

TEST(RouteMechanism, InvalidOrExpiredViolationCannotCreateNewPhysicalAuthority)
{
  for (const int observed:{90,101}) {
    auto g=graph({0.,10.});edge(*g,0,1,10.,true,true,observed);
    graphnav_msgs::msg::KeyValue witness;witness.key="physical_violation_observed";witness.value={1.f};
    g->edges[0].properties.push_back(witness);
    auto p=planner();p.update_graph(g);Eigen::Vector3d goal(10.,0.,0.);
    EXPECT_FALSE(p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp)).empty());
  }
}

TEST(RouteMechanism, DirectedConnectionDoesNotInventReverseReachability)
{
  auto g=graph({0.,10.});edge(*g,1,0,10.);
  auto p=planner();p.update_graph(g);Eigen::Vector3d forward(10.,0.,0.);
  EXPECT_TRUE(p.plan_to_goal(forward,.5,rclcpp::Time(g->header.stamp)).empty());
  g->current_node_idx=1;p.update_graph(g);Eigen::Vector3d reverse(0.,0.,0.);
  EXPECT_FALSE(p.plan_to_goal(reverse,.5,rclcpp::Time(g->header.stamp)).empty());
}

TEST(RouteMechanism, ReverseConnectionCannotReopenMeasuredBlockedDirection)
{
  for (bool reverse_order:{false,true}) {
    auto g=graph({0.,10.});edge(*g,0,1,10.,true,true);
    graphnav_msgs::msg::KeyValue witness;witness.key="physical_violation_observed";witness.value={1.f};
    g->edges[0].properties.push_back(witness);
    edge(*g,1,0,10.,false,false,100,0.f);
    if (reverse_order) std::reverse(g->edges.begin(),g->edges.end());
    auto p=planner();p.update_graph(g);Eigen::Vector3d forward(10.,0.,0.);
    EXPECT_TRUE(p.plan_to_goal(forward,.5,rclcpp::Time(g->header.stamp)).empty());
    g->current_node_idx=1;p.update_graph(g);Eigen::Vector3d retreat(0.,0.,0.);
    EXPECT_FALSE(p.plan_to_goal(retreat,.5,rclcpp::Time(g->header.stamp)).empty());
  }
}

TEST(RouteMechanism, RejectedLocalEdgeUsesAlternateGraphRoute)
{
  auto g = graph({0., 10., 20., 0., 20.});
  g->nodes[3].pose.position.y = 5.;
  g->nodes[4].pose.position.y = 5.;
  edge(*g, 0, 1, 10.);
  edge(*g, 1, 2, 10.);
  edge(*g, 0, 3, 6.);
  edge(*g, 3, 4, 20.);
  edge(*g, 4, 2, 6.);
  auto p = planner();
  p.set_rejected_edges({{Eigen::Vector2d(0., 0.), Eigen::Vector2d(10., 0.)}});
  p.update_graph(g);
  Eigen::Vector3d goal(20., 0., 0.);
  const auto path = p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp));
  ASSERT_FALSE(path.empty());
  ASSERT_TRUE(p.route_complete());
  EXPECT_NEAR(path.front().x(), 0., 1e-9);
  EXPECT_TRUE(std::any_of(path.begin(), path.end(), [](const auto& point) {
    return std::abs(point.y() - 5.) < 1e-9;
  }));
  EXPECT_FALSE(std::any_of(path.begin(), path.end(), [](const auto& point) {
    return std::abs(point.x() - 10.) < 1e-9 && std::abs(point.y()) < 1e-9;
  }));
}

TEST(RouteMechanism, StaleAndUnknownBlocksCannotEraseKnownTopology)
{
  for (const auto assessment : {std::make_pair(90, .8f), std::make_pair(100, 0.f)})
  {
    auto g = graph({0., 10.});
    edge(*g, 0, 1, 10., true, false, assessment.first, assessment.second);
    auto p = planner(); p.update_graph(g);
    Eigen::Vector3d goal(10., 0., 0.);
    EXPECT_FALSE(p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp)).empty());
  }
}

TEST(RouteMechanism, InvalidGraphWeightsAreRejected)
{
  for (const double cost : {-1., std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()})
  {
    auto g = graph({0., 10.});
    edge(*g, 0, 1, cost);
    auto p = planner(); p.update_graph(g);
    Eigen::Vector3d goal(10., 0., 0.);
    EXPECT_TRUE(p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp)).empty());
  }
}

TEST(RouteMechanism, EmptyGraphAndCoincidentGoalDoNotDereferenceMissingFrontier)
{
  auto p = planner();
  auto empty = graph({});
  p.update_graph(empty);
  Eigen::Vector3d goal(0., 0., 0.);
  EXPECT_TRUE(p.plan_to_goal(goal, .5, rclcpp::Time(empty->header.stamp)).empty());
  auto single = graph({0.});
  p.update_graph(single);
  const auto path = p.plan_to_goal(goal, .5, rclcpp::Time(single->header.stamp));
  for (const auto& point : path) EXPECT_TRUE(point.allFinite());
  EXPECT_FALSE(p.get_score_visualization(rclcpp::Time(single->header.stamp), "spot1/odom", true).markers.empty());
}

TEST(RouteMechanism, InvalidAssessmentConfidenceCannotDeleteTopology)
{
  for (float confidence : {std::numeric_limits<float>::quiet_NaN(), 2.f})
  {
    auto g = graph({0., 10.});
    edge(*g, 0, 1, 10., true, false, 100, confidence);
    auto p = planner(); p.update_graph(g);
    Eigen::Vector3d goal(10., 0., 0.);
    EXPECT_FALSE(p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp)).empty());
  }
}

TEST(RouteMechanism, StaleDynamicRiskCannotRemoveGoalAttachment)
{
  auto g = graph({0., 10.});
  edge(*g, 0, 1, 10.);
  g->nodes[1].robot_traversability[0].dynamic_risk = 1.f;
  g->nodes[1].robot_traversability[0].observed_at.sec = 90;
  auto p = planner(); p.update_graph(g);
  Eigen::Vector3d goal(10., 0., 0.);
  EXPECT_FALSE(p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp)).empty());
  EXPECT_TRUE(p.route_complete());
}

TEST(RouteMechanism, InvalidFrontierPointDoesNotMakePrefixNonfinite)
{
  auto g = graph({0., 2.});
  edge(*g, 0, 1, 2.);
  frontier(*g, 1);
  auto bad = g->nodes[1].trav_properties[0].frontier_points[0];
  bad.x = std::numeric_limits<double>::quiet_NaN();
  g->nodes[1].trav_properties[0].frontier_points.push_back(bad);
  auto p = planner(); p.update_graph(g);
  Eigen::Vector3d goal(50., 0., 0.);
  const auto path = p.plan_to_goal(goal, .5, rclcpp::Time(g->header.stamp));
  ASSERT_FALSE(path.empty());
  for (const auto& point : path) EXPECT_TRUE(point.allFinite());
}

TEST(DestinationFence, FreshDestinationsRemainUsableAfterClear)
{
  graphnav_planner::DestinationFence fence;
  fence.clear(100);
  EXPECT_FALSE(fence.accept(99, 105));
  EXPECT_FALSE(fence.accept(100, 105));
  EXPECT_TRUE(fence.accept(101, 105));
  EXPECT_TRUE(fence.accept(104, 105));
}

TEST(DestinationFence, DuplicateOrReorderedGoalsCannotReplaceCurrentDestination)
{
  graphnav_planner::DestinationFence fence;
  EXPECT_TRUE(fence.accept(200, 205));
  EXPECT_FALSE(fence.accept(200, 205));
  EXPECT_FALSE(fence.accept(199, 205));
  EXPECT_TRUE(fence.accept(201, 205));
}

TEST(DestinationFence, FutureOrInvalidGoalsCannotPoisonNextValidGoal)
{
  graphnav_planner::DestinationFence fence;
  EXPECT_FALSE(fence.accept(300, 250));
  EXPECT_FALSE(fence.accept(0, 250));
  EXPECT_FALSE(fence.accept(-1, 250));
  EXPECT_FALSE(fence.accept(1, 0));
  EXPECT_TRUE(fence.accept(249, 250));
}

TEST(DestinationFence, ReleaseRejectsEarlierAcceptedGoalButAllowsNewOne)
{
  graphnav_planner::DestinationFence fence;
  EXPECT_TRUE(fence.accept(101, 105));
  fence.clear(110);
  EXPECT_FALSE(fence.accept(101, 115));
  EXPECT_TRUE(fence.accept(111, 115));
}

TEST(DestinationFence, ClearIsAClockBoundaryAndFutureOldEpochIsRejected)
{
  graphnav_planner::DestinationFence fence;
  fence.clear(1000);
  EXPECT_TRUE(fence.accept(1001, 1002));
  fence.clear(10);
  EXPECT_FALSE(fence.accept(1001, 12));
  EXPECT_TRUE(fence.accept(11, 12));
}

TEST(DestinationFence, RepeatedClearCannotReopenPriorGoal)
{
  graphnav_planner::DestinationFence fence;
  fence.clear(100);
  EXPECT_TRUE(fence.accept(101, 102));
  fence.clear(103);
  fence.clear(104);
  EXPECT_FALSE(fence.accept(101, 105));
  EXPECT_TRUE(fence.accept(105, 105));
}
