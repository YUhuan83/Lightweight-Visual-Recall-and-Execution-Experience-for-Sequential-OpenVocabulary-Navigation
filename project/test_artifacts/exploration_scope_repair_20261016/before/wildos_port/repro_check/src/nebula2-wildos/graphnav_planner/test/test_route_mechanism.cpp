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

using graphnav_planner::Planner;
using Graph = graphnav_msgs::msg::NavigationGraph;

TEST(ExplorationTarget, ScoreUpdatesCannotReplaceOwnedDestination)
{
  graphnav_planner::ExplorationTarget target;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map"; goal.pose.position.x = 20.;
  ASSERT_TRUE(target.begin("a", goal, .4));
  const auto identity = target.identity();
  goal.pose.position.x = 30.;
  EXPECT_FALSE(target.begin("b", goal, .8));
  EXPECT_EQ(target.identity(), identity);
  EXPECT_EQ(target.uuid(), "a");
  EXPECT_DOUBLE_EQ(target.goal().pose.position.x, 20.);
  target.clear();
  ASSERT_TRUE(target.begin("a", goal, .8));
  EXPECT_NE(target.identity(), identity);
  EXPECT_FALSE(target.acknowledge(identity));
  graphnav_planner::ExplorationTarget restarted;
  ASSERT_TRUE(restarted.begin("a", goal, .8));
  EXPECT_NE(restarted.identity(), target.identity());
}

TEST(ExplorationTarget, NeedsCorrelatedExecutionAckAndFreshSettledOdometry)
{
  graphnav_planner::ExplorationTarget target;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map"; goal.pose.position.x = 20.;
  ASSERT_TRUE(target.begin("a", goal, 0.));
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map"; odom.pose.pose = target.goal().pose;
  odom.header.stamp.sec = 10;
  EXPECT_FALSE(target.complete(odom, 10000000000LL, 1.));
  EXPECT_FALSE(target.acknowledge("b"));
  ASSERT_TRUE(target.acknowledge(target.identity()));
  EXPECT_FALSE(target.complete(odom, 10000000000LL, 1.));
  EXPECT_FALSE(target.complete(odom, 10400000000LL, 1.)); // repeated acquisition
  odom.header.stamp.sec = 11;
  EXPECT_FALSE(target.complete(odom, 11000000000LL, 1.));
  odom.header.stamp.nanosec = 500000000;
  EXPECT_TRUE(target.complete(odom, 11500000000LL, 1.));
  EXPECT_FALSE(target.complete(odom, 12500000000LL, 1.)); // stale clears dwell
  odom.header.stamp.sec = 13; odom.header.stamp.nanosec = 0;
  EXPECT_FALSE(target.complete(odom, 13000000000LL, 1.));
  odom.header.stamp.sec = 15; odom.header.frame_id = "other";
  EXPECT_FALSE(target.complete(odom, 15000000000LL, 1.));
  odom.header.frame_id = "map"; odom.pose.pose.position.x = 15.;
  EXPECT_FALSE(target.complete(odom, 15000000000LL, 1.)); // graph's 6m is not arrival
  odom.pose.pose.position.x = 20.; odom.twist.twist.linear.x = .3;
  EXPECT_FALSE(target.complete(odom, 15000000000LL, 1.));
  odom.twist.twist.linear.x = 0.; odom.pose.pose.orientation.w = 0.;
  EXPECT_FALSE(target.complete(odom, 15000000000LL, 1.));
  odom.pose.pose.orientation.w = 1.;
  EXPECT_FALSE(target.complete(odom, 14000000000LL, 1.)); // future
}

TEST(ExplorationTarget, MissingHeadingStillRequiresExecutedSettledViewAndClearFencesAck)
{
  graphnav_planner::ExplorationTarget target;
  geometry_msgs::msg::PoseStamped goal; goal.header.frame_id = "map";
  ASSERT_TRUE(target.begin("a", goal, std::nullopt));
  const auto old = target.identity();
  ASSERT_TRUE(target.acknowledge(old));
  nav_msgs::msg::Odometry odom; odom.header.frame_id = "map";
  odom.pose.pose.orientation.w = 1.; odom.header.stamp.sec = 10;
  EXPECT_FALSE(target.complete(odom, 10000000000LL, 1.));
  odom.header.stamp.sec = 12;
  EXPECT_TRUE(target.complete(odom, 12000000000LL, 1.));
  target.clear();
  ASSERT_TRUE(target.begin("a", goal, std::nullopt));
  EXPECT_FALSE(target.acknowledge(old));
  EXPECT_FALSE(target.complete(odom, 12000000000LL, 1.));
}

TEST(ExplorationBackoff, RouteIdentityFencesDelayedRefusalFromAnotherDestination)
{
  graphnav_planner::ExplorationBackoff policy;
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = "map"; message.header.stamp.sec = 10;
  policy.remember(message.header, message.pose.position, message.pose.position, "a:1");
  EXPECT_EQ(policy.identity(message, 11000000000LL), "a:1");
  message.header.stamp.sec = 11;
  EXPECT_FALSE(policy.identity(message, 11000000000LL));
  message.header.stamp.sec = 10; message.pose.position.x = 1.;
  EXPECT_FALSE(policy.identity(message, 11000000000LL));
  message.pose.position.x = 0.;
  EXPECT_FALSE(policy.identity(message, 9000000000LL));
  EXPECT_FALSE(policy.identity(message, 71000000000LL));
  policy.clear(); EXPECT_FALSE(policy.identity(message, 11000000000LL));
}

TEST(ExplorationObservation, SelectsFiniteTraversableHeadingAndRequiresSettledView)
{
  graphnav_msgs::msg::Node node;
  node.heading_candidates.resize(3);
  node.heading_candidates[0].traversable = true;
  node.heading_candidates[0].heading = .6f;
  node.heading_candidates[0].predicted_utility = .8f;
  node.heading_candidates[1].predicted_utility = 1.f;  // infeasible
  node.heading_candidates[2].traversable = true;
  node.heading_candidates[2].predicted_utility = std::numeric_limits<float>::quiet_NaN();
  ASSERT_TRUE(graphnav_planner::exploration_heading(node));
  EXPECT_NEAR(*graphnav_planner::exploration_heading(node), .6, 1e-6);
  graphnav_planner::ExplorationObservation view;
  EXPECT_FALSE(view.complete(1000000000LL, true, 0., .6, 0.));
  EXPECT_FALSE(view.complete(2000000000LL, true, .6, .6, .5));
  EXPECT_FALSE(view.complete(3000000000LL, true, .6, .6, 0.));
  EXPECT_FALSE(view.complete(3000000000LL, true, .6, .6, 0.));  // paused
  EXPECT_FALSE(view.complete(4000000000LL, true, .6, .6, 0.));
  EXPECT_TRUE(view.complete(4500000000LL, true, .6, .6, 0.));
  EXPECT_FALSE(view.complete(5000000000LL, false, .6, .6, 0.));
  EXPECT_FALSE(view.complete(6000000000LL, true, .6, .6, 0.));
}

TEST(GraphScoreOverlay, ExactSnapshotRetainsTopologyAndAppliesScores)
{
  graphnav_planner::GraphScoreOverlay overlay;
  auto raw = std::make_shared<Graph>();
  raw->header.frame_id = "spot1/odom"; raw->header.stamp.sec = 10;
  raw->nodes.resize(2); raw->nodes[1].uuid.id[0] = 1;
  raw->edges.resize(1); raw->edges[0].to_idx = 1;
  overlay.remember(raw);
  auto scores = *raw; scores.edges.clear();
  scores.nodes[0].heading_candidates.resize(1);
  auto combined = overlay.combine(scores);
  ASSERT_TRUE(combined); EXPECT_EQ(combined->edges.size(), 1U);
  EXPECT_EQ(combined->nodes[0].heading_candidates.size(), 1U);
  EXPECT_TRUE(raw->nodes[0].heading_candidates.empty());
}

TEST(GraphScoreOverlay, DifferentStampOrderingPoseAndFrameCannotMixIndices)
{
  graphnav_planner::GraphScoreOverlay overlay;
  auto raw = std::make_shared<Graph>();
  raw->header.frame_id = "spot1/odom"; raw->header.stamp.sec = 10;
  raw->nodes.resize(2); raw->nodes[1].uuid.id[0] = 1; raw->edges.resize(1);
  overlay.remember(raw);
  auto scores = *raw; scores.edges.clear();
  scores.header.stamp.sec = 11; EXPECT_FALSE(overlay.combine(scores));
  scores.header = raw->header; std::swap(scores.nodes[0], scores.nodes[1]);
  EXPECT_FALSE(overlay.combine(scores));
  scores.nodes = raw->nodes; scores.nodes[0].pose.position.x = 1.;
  EXPECT_FALSE(overlay.combine(scores));
  scores.nodes = raw->nodes; scores.header.frame_id = "other";
  EXPECT_FALSE(overlay.combine(scores));
}

TEST(GraphScoreOverlay, SnapshotStorageIsBounded)
{
  graphnav_planner::GraphScoreOverlay overlay;
  for (int n=0; n<20; ++n) { auto raw=std::make_shared<Graph>(); raw->header.stamp.sec=n; overlay.remember(raw); }
  EXPECT_EQ(overlay.size(), 8U);
  Graph old; old.header.stamp.sec=0; EXPECT_FALSE(overlay.combine(old));
}

TEST(ExplorationBackoff, MatchesIssuedRouteAndExpires)
{
  graphnav_planner::ExplorationBackoff policy;
  geometry_msgs::msg::PoseStamped refusal;
  refusal.header.frame_id = "spot1/odom";
  refusal.header.stamp.sec = 100;
  refusal.pose.position.x = 5.0;
  geometry_msgs::msg::Point frontier;
  frontier.x = 20.;
  policy.remember(refusal.header, refusal.pose.position, frontier, "frontier-a");
  EXPECT_TRUE(policy.reject(refusal, 110000000000LL, true));
  EXPECT_TRUE(policy.blocked(frontier, 111000000000LL, 6.));
  auto other = frontier; other.y = 10.;
  EXPECT_TRUE(policy.blocked(other, 111000000000LL, 6.));
  EXPECT_TRUE(policy.blocked(other, 111000000000LL, graphnav_planner::rejected_frontier_radius_m));
  auto outside = frontier; outside.y = graphnav_planner::rejected_frontier_radius_m + .1;
  EXPECT_FALSE(policy.blocked(outside, 111000000000LL, 6.));
  EXPECT_TRUE(policy.blocked(frontier, 229000000000LL, 6.));
  EXPECT_FALSE(policy.blocked(frontier, 230000000000LL, 6.));
}

TEST(ExplorationBackoff, RejectingAtVehiclePositionReturnsTheFailedGraphEdgeTemporarily)
{
  graphnav_planner::ExplorationBackoff policy;
  geometry_msgs::msg::PoseStamped refusal;
  refusal.header.frame_id = "spot1/odom";
  refusal.header.stamp.sec = 100;
  refusal.pose.position.x = 20.;
  geometry_msgs::msg::Point frontier;
  frontier.x = 30.;
  std::vector<geometry_msgs::msg::Point> path(3);
  path[0].x = 0.; path[1].x = 10.; path[2].x = 20.;
  policy.remember(refusal.header, refusal.pose.position, frontier, "frontier-a", path);

  geometry_msgs::msg::Point vehicle;
  vehicle.x = 8.; vehicle.y = 1.;
  EXPECT_TRUE(policy.reject(refusal, 110000000000LL, true, vehicle));
  const auto active = policy.active_edges(111000000000LL);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_NEAR(active[0].first.x(), 0., 1e-9);
  EXPECT_NEAR(active[0].second.x(), 10., 1e-9);
  EXPECT_TRUE(policy.blocked(vehicle, 111000000000LL, 1.0));
  geometry_msgs::msg::Point outside_stall_region;
  outside_stall_region.x = 17.1;
  EXPECT_FALSE(policy.blocked(outside_stall_region, 111000000000LL, 1.0));
  EXPECT_TRUE(policy.active_edges(229000000000LL).size() == 1u);
  EXPECT_TRUE(policy.active_edges(230000000000LL).empty());
}

TEST(ExplorationBackoff, DistantVehiclePositionDoesNotBlacklistAnUnrelatedEdge)
{
  graphnav_planner::ExplorationBackoff policy;
  geometry_msgs::msg::PoseStamped refusal;
  refusal.header.frame_id = "spot1/odom";
  refusal.header.stamp.sec = 100;
  refusal.pose.position.x = 20.;
  geometry_msgs::msg::Point destination;
  std::vector<geometry_msgs::msg::Point> path(2);
  path[0].x = 0.; path[1].x = 10.;
  policy.remember(refusal.header, refusal.pose.position, destination, "frontier-a", path);
  geometry_msgs::msg::Point vehicle;
  vehicle.x = 100.;
  EXPECT_TRUE(policy.reject(refusal, 110000000000LL, true, vehicle));
  EXPECT_TRUE(policy.active_edges(111000000000LL).empty());
}

TEST(ExplorationUtility, PrefersObservedTraversableFrontierToUnknownBrush)
{
  const double observed_road = graphnav_planner::exploration_candidate_utility(
      0.65, 20.0, 0, 0.8, 0.9);
  const double unknown_frontier = graphnav_planner::exploration_candidate_utility(
      0.95, 5.0, 0, 0.0, 0.0);
  EXPECT_GT(observed_road, unknown_frontier);
  EXPECT_TRUE(std::isfinite(unknown_frontier));
}

TEST(ExplorationUtility, UnknownFrontiersRemainSelectableAndInvalidInputsAreRejected)
{
  EXPECT_TRUE(std::isfinite(graphnav_planner::exploration_candidate_utility(
      0.8, 10.0, 0, 0.0, 0.0)));
  EXPECT_TRUE(std::isinf(graphnav_planner::exploration_candidate_utility(
      std::numeric_limits<double>::quiet_NaN(), 10.0, 0, 1.0, 1.0)));
}

TEST(ExplorationBackoff, ExplicitGoalsAndQueryChangesRejectOldFeedback)
{
  graphnav_planner::ExplorationBackoff policy;
  geometry_msgs::msg::PoseStamped refusal;
  refusal.header.frame_id = "spot1/odom";
  refusal.header.stamp.sec = 100;
  policy.remember(refusal.header, refusal.pose.position, refusal.pose.position, "a");
  EXPECT_FALSE(policy.reject(refusal, 110000000000LL, false));
  policy.clear();
  EXPECT_FALSE(policy.reject(refusal, 110000000000LL, true));
}

TEST(ExplorationBackoff, RefusesStaleFutureOrUncorrelatedEndpoints)
{
  graphnav_planner::ExplorationBackoff policy;
  geometry_msgs::msg::PoseStamped refusal;
  refusal.header.frame_id = "spot1/odom";
  refusal.header.stamp.sec = 100;
  policy.remember(refusal.header, refusal.pose.position, refusal.pose.position, "a");
  EXPECT_FALSE(policy.reject(refusal, 99000000000LL, true));
  EXPECT_FALSE(policy.reject(refusal, 161000000000LL, true));
  refusal.pose.position.x = 1.;
  EXPECT_FALSE(policy.reject(refusal, 110000000000LL, true));
  refusal.pose.position.x = 0.; refusal.header.frame_id = "map";
  EXPECT_FALSE(policy.reject(refusal, 110000000000LL, true));
}

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

