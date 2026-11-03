#include <gtest/gtest.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "graphnav_mapper/mapper.hpp"

class FrontierCoverage : public ::testing::Test {
protected:
  static void SetUpTestSuite() { if(!rclcpp::ok()) rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::Clock::SharedPtr clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer{clock};
  graphnav_mapper::Mapper mapper{buffer,rclcpp::get_logger("coverage_test")};

  void SetUp() override {
    auto pg=std::make_shared<pose_graph_msgs::msg::PoseGraph>();
    pg->header.frame_id="spot1/odom";pg->nodes.resize(1);
    pg->nodes[0].pose.orientation.w=1.;pg->current_node=pg->nodes[0].uuid;
    mapper.pose_graph_update(pg);
    auto odom=std::make_shared<nav_msgs::msg::Odometry>();
    odom->header=pg->header;odom->header.stamp.sec=10;
    odom->pose.pose.position.x=100.;odom->pose.pose.position.y=100.;
    odom->pose.pose.orientation.w=1.;mapper.odometry_update(odom);
    mapper.traversability_update_nb_nodes_sample_=0;
  }

  void update(double x,double y,double size=6.) {
    grid_map::GridMap map({"traversability","unknown","elevation"});
    map.setFrameId("spot1/odom");map.setGeometry({size,size},.5,{x,y});
    map.setTimestamp(30000000000ULL);
    for(const auto& name:map.getLayers()) map[name].setZero();
    mapper.traversability_update(grid_map::GridMapRosConverter::toMessage(map));
  }
};

TEST_F(FrontierCoverage, HistoricalCoverageRetiresOldExitOutsideTheCurrentMap) {
  auto& graph=mapper.get_graph();
  graphnav::NavigationGraph::NodeProperties cover;cover.explored_radius=11.7;
  graph.add_node({4.3,0.,0.},cover);
  graphnav::NavigationGraph::NodeProperties properties;
  auto& trav=properties.trav_properties["default"];trav.is_frontier=true;
  trav.frontier_points={{0.,0.,0.},{0.,20.,0.}};
  const auto owner=graph.add_node({0.,0.,0.},properties);
  update(100.,100.);
  const auto& after=graph.get_node_properties(owner).trav_properties.at("default");
  ASSERT_TRUE(after.is_frontier);ASSERT_EQ(after.frontier_points.size(),1u);
  EXPECT_DOUBLE_EQ(after.frontier_points[0].y(),20.);
  cover.explored_radius=30.;
  const auto covers=graph.get_nodes_in_radius({4.3,0.,0.},.01);
  ASSERT_EQ(covers.size(),1u);graph.set_node_properties(covers[0],cover);
  update(100.,100.);
  EXPECT_FALSE(graph.get_node_properties(owner).trav_properties.at("default").is_frontier);
  EXPECT_TRUE(graph.get_node_properties(owner).trav_properties.at("default").frontier_points.empty());
}

TEST_F(FrontierCoverage, LargeObservationRadiusPreventsNewCoveredFrontiers) {
  auto& graph=mapper.get_graph();
  graphnav::NavigationGraph::NodeProperties cover;cover.explored_radius=20.;
  graph.add_node({0.,7.,0.},cover);
  graphnav::NavigationGraph::NodeProperties properties;
  auto& trav=properties.trav_properties["default"];trav.is_frontier=true;trav.free_radius=4.;
  trav.frontier_points={{0.,30.,0.}};
  const auto owner=graph.add_node({0.,0.,0.},properties);
  update(0.,0.);
  const auto& after=graph.get_node_properties(owner).trav_properties.at("default");
  ASSERT_TRUE(after.is_frontier);ASSERT_EQ(after.frontier_points.size(),1u);
  EXPECT_DOUBLE_EQ(after.frontier_points[0].y(),30.);
}
