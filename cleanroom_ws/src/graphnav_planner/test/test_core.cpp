#include <gtest/gtest.h>
#include "graphnav_planner/planner.hpp"
TEST(GraphPlanning, KnownGoalUsesConnectedMeasuredGraph) {
  graphnav_planner::Planner planner(rclcpp::get_logger("core_test"));
  planner.set_trav_class("default");
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
  planner.update_graph(graph);
  Eigen::Vector3d goal(10.,0.,52.);
  auto path=planner.plan_to_goal(goal,3.,rclcpp::Time(10,0));
  ASSERT_FALSE(path.empty());EXPECT_TRUE(planner.route_complete());
  EXPECT_NEAR((path.back()-goal).norm(),0.,1e-5);
}
