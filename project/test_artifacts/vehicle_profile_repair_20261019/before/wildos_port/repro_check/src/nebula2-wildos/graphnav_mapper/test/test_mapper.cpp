#include <algorithm>
#include <limits>
#include <future>
#include <thread>
#include <gtest/gtest.h>

#include "graphnav_mapper/mapper.hpp"
#include "graphnav_mapper/terrain_metrics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "pose_graph_msgs/msg/pose_graph.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

static bool publish_on_failure_ = false;

TEST(TerrainMetrics, SmoothRampIsSlopeNotStepOrRoughness) {
  std::vector<Eigen::Vector3d> points;
  for (int x=-1; x<=1; ++x) for (int y=-1; y<=1; ++y)
    points.emplace_back(.5*x, .5*y, .2*.5*x+.1*.5*y);
  const auto metrics=graphnav_mapper::terrain_metrics(points);
  ASSERT_TRUE(metrics.observed);
  EXPECT_NEAR(metrics.slope, std::hypot(.2,.1), 1e-8);
  EXPECT_LT(metrics.step, 1e-8); EXPECT_LT(metrics.roughness, 1e-8);
}
TEST(TerrainMetrics, SharpHeightChangeAndMissingSamplesAreNotFlatRoad) {
  std::vector<Eigen::Vector3d> points;
  for (int x=-1; x<=1; ++x) for (int y=-1; y<=1; ++y)
    points.emplace_back(.5*x, .5*y, x>0?.4:0.);
  const auto metrics=graphnav_mapper::terrain_metrics(points);
  ASSERT_TRUE(metrics.observed); EXPECT_GT(metrics.step, .1);
  EXPECT_GT(metrics.roughness, .08);
  EXPECT_FALSE(graphnav_mapper::terrain_metrics({{0.,0.,0.},{.5,0.,0.}}).observed);
}

class MapperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::Logger logger = rclcpp::get_logger("MapperTest");
    logger.set_level(rclcpp::Logger::Level::Debug);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    mapper_ = std::make_shared<graphnav_mapper::Mapper>(*tf_buffer_, logger);

    mapper_->traversability_classes_["default"].layer_name = "traversability";
    mapper_->traversability_classes_["default"].obstacle_threshold = 0.001;
    mapper_->traversability_classes_["default"].radius = 1.0;

    stamp_ = rclcpp::Time(1, 0, RCL_ROS_TIME);

    grid_map_.setFrameId("map");
    grid_map_.setGeometry(grid_map::Length(50.0, 50.0), 0.1, grid_map::Position(10.0, 5.0));
    grid_map_.add("unknown", 0.0);
    grid_map_.add("traversability", 0.0);
    grid_map_.add("elevation", 0.0);
    grid_map_.setTimestamp(stamp_.nanoseconds());
    dbg_map_ = nullptr;

    pose_graph_ = std::make_shared<pose_graph_msgs::msg::PoseGraph>();
    pose_graph_->header.frame_id = "map";
    pose_graph_->header.stamp = stamp_;
    pose_graph_->nodes.resize(1);
    pose_graph_->nodes[0].pose.orientation.w = 1.0;
    pose_graph_->nodes[0].uuid.id[0] = 1;
    pose_graph_->current_node = pose_graph_->nodes[0].uuid;
    pose_graph_->edges.resize(0);

    mapper_->pose_graph_update(pose_graph_);
  }

  void TearDown() override
  {
    if (::testing::Test::HasFailure() && publish_on_failure_)
    {
      std::cout << "Test failed, publishing debug information..." << std::endl;
      std::cout << "Test name: " << ::testing::UnitTest::GetInstance()->current_test_info()->name() << std::endl;
      auto node = rclcpp::Node::make_shared("test_node");
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.transient_local();
      auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>("debug_map", qos);
      if (dbg_map_)
      {
        publisher->publish(*dbg_map_);
      }
      auto graph_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("graph", qos);
      graph_publisher->publish(mapper_->get_visualization(stamp_, true));
      std::cout << "Press Enter to continue..." << std::endl;
      std::cin.get();
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<graphnav_mapper::Mapper> mapper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  grid_map::GridMap grid_map_;
  std::shared_ptr<grid_map_msgs::msg::GridMap> dbg_map_;
  std::shared_ptr<pose_graph_msgs::msg::PoseGraph> pose_graph_;
  rclcpp::Time stamp_;

  auto measured_connection()
  {
    mapper_->traversability_update_nb_nodes_sample_ = 0;
    grid_map_.add("elevation_valid",1.f);
    graphnav::NavigationGraph::NodeProperties properties;
    properties.trav_properties["default"].free_radius=2.;
    properties.trav_properties["default"].visited=true;
    auto& graph=mapper_->get_graph();
    const auto a=graph.add_node(Eigen::Vector3d(2.,2.,0.),properties);
    const auto b=graph.add_node(Eigen::Vector3d(6.,2.,0.),properties);
    graphnav::NavigationGraph::EdgeProperties connection;
    connection.trav_properties["default"].cost=4.;
    connection.trav_properties["default"].traversed=true;
    graph.add_edge(a,b,connection);
    return std::make_pair(a,b);
  }

  void update_measured_map(int seconds=1)
  {
    grid_map_.setTimestamp(static_cast<uint64_t>(seconds)*1000000000ULL);
    mapper_->traversability_update(grid_map::GridMapRosConverter::toMessage(grid_map_));
  }

  template<typename Properties>
  static float evidence(const Properties& properties,const std::string& key)
  {
    const auto value=properties.metadata.find(key);
    return value==properties.metadata.end() || value->second.empty() ? 0.f : value->second[0];
  }
};

TEST_F(MapperTest, OdometryUpdateTest)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->pose.pose.position.x = 1.0;
  odom->pose.pose.position.y = 2.0;
  odom->pose.pose.orientation.w = 1.0;

  mapper_->odometry_update(odom);

  // Add assertions to verify the state of the mapper after the odometry update
  // For example, check if the current node is set correctly
  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, AcquisitionTransformCanArriveAfterItsMap)
{
  grid_map_.setGeometry(grid_map::Length(10.0, 10.0), .5);
  for (const auto& layer : grid_map_.getLayers()) grid_map_[layer].setZero();
  grid_map_.setFrameId("terrain");
  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  msg->header.stamp = stamp_;
  geometry_msgs::msg::TransformStamped old_tf;
  old_tf.header.frame_id = "map";
  old_tf.child_frame_id = "terrain";
  old_tf.header.stamp = rclcpp::Time(0, 500000000, RCL_ROS_TIME);
  old_tf.transform.rotation.w = 1.;
  ASSERT_TRUE(tf_buffer_->setTransform(old_tf, "test"));
  tf_buffer_->setUsingDedicatedThread(true);
  auto exact_tf = old_tf;
  exact_tf.header.stamp = stamp_;
  exact_tf.transform.translation.x = 2.;
  auto delivery = std::async(std::launch::async, [buffer = tf_buffer_, exact_tf]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return buffer->setTransform(exact_tf, "test");
  });
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update(msg, dbg_map_);
  EXPECT_TRUE(delivery.get());
  EXPECT_EQ(dbg_map_->header.stamp, msg->header.stamp);
  EXPECT_EQ(dbg_map_->header.frame_id, "terrain");
  EXPECT_FALSE(dbg_map_->layers.empty());
}

TEST_F(MapperTest, MissingAcquisitionTransformDoesNotSubstituteLatestPose)
{
  grid_map_.setGeometry(grid_map::Length(10.0, 10.0), .5);
  for (const auto& layer : grid_map_.getLayers()) grid_map_[layer].setZero();
  grid_map_.setFrameId("terrain");
  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  msg->header.stamp = stamp_;
  geometry_msgs::msg::TransformStamped old_tf;
  old_tf.header.frame_id = "map";
  old_tf.child_frame_id = "terrain";
  old_tf.header.stamp = rclcpp::Time(0, 500000000, RCL_ROS_TIME);
  old_tf.transform.rotation.w = 1.;
  ASSERT_TRUE(tf_buffer_->setTransform(old_tf, "test"));
  tf_buffer_->setUsingDedicatedThread(true);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  const auto before = mapper_->get_graph().get_all_nodes().size();
  mapper_->traversability_update(msg, dbg_map_);
  EXPECT_TRUE(dbg_map_->layers.empty());
  EXPECT_EQ(mapper_->get_graph().get_all_nodes().size(), before);
  old_tf.header.stamp = stamp_;
  ASSERT_TRUE(tf_buffer_->setTransform(old_tf, "test"));
  mapper_->traversability_update(msg, dbg_map_);
  EXPECT_FALSE(dbg_map_->layers.empty());
}

TEST_F(MapperTest, TraversabilityUpdateCreatesNodesInFreeSpace)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10)
    {
      grid_map_.at("traversability", *it) = 1.0;
    }
  }
  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update(msg, dbg_map_);

  // ASSERT_TRUE(false);

  auto nodes = mapper_->get_graph().get_all_nodes();
  EXPECT_GT(nodes.size(), 0);  // nodes are created
  for (const auto& node : nodes)
  {
    auto pos = mapper_->get_graph().get_node_position(node);
    EXPECT_TRUE(grid_map_.isInside(pos.head<2>()));
    EXPECT_LT(grid_map_.atPosition("unknown", pos.head<2>()), 0.5);
    EXPECT_LT(grid_map_.atPosition("traversability", pos.head<2>()), 0.5);
  }

  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdatesNodeProperties)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10)
    {
      grid_map_.at("traversability", *it) = 1.0;
    }
  }

  // add a node in the free space
  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties init_properties;
  init_properties.trav_properties["default"].free_radius = mapper_->node_free_radius_limit_;
  init_properties.explored_radius = 2.0;
  auto node_to_update = graph.add_node(Eigen::Vector3d(5.0, 8.0, 0.0), init_properties);
  auto node_to_delete = graph.add_node(Eigen::Vector3d(15.0, 15.0, 0.0), init_properties);
  auto node_to_leave = graph.add_node(Eigen::Vector3d(19.0, 0.0, 0.0), init_properties);
  // std::cout << "node_to_update: " << node_to_update.to_string() << std::endl;
  // std::cout << "node_to_delete: " << node_to_delete.to_string() << std::endl;
  // std::cout << "node_to_leave: " << node_to_leave.to_string() << std::endl;

  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update(msg, dbg_map_);

  // updated free radius & explored radius
  EXPECT_TRUE(graph.node_exists(node_to_update));
  auto node_prop = graph.get_node_properties(node_to_update);
  EXPECT_EQ(node_prop.trav_properties.size(), 1);
  EXPECT_NEAR(node_prop.trav_properties["default"].free_radius, 10 - 8, 0.2);
  EXPECT_NEAR(node_prop.explored_radius, 20 - 5, 0.2);

  // unchanged node
  EXPECT_TRUE(graph.node_exists(node_to_leave));
  node_prop = graph.get_node_properties(node_to_leave);
  EXPECT_EQ(node_prop.trav_properties.size(), 1);
  EXPECT_NEAR(node_prop.trav_properties["default"].free_radius, init_properties.trav_properties["default"].free_radius,
              0.01);
  EXPECT_NEAR(node_prop.explored_radius, init_properties.explored_radius, 0.01);

  // deleted node (too close to obstacle)
  EXPECT_FALSE(graph.node_exists(node_to_delete));

  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdatesEdges)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10 && position.y() < 11)
    {
      grid_map_.at("traversability", *it) = 1.0;
    }
  }

  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties init_properties;
  init_properties.trav_properties["default"].free_radius = 1.0;
  init_properties.explored_radius = 2.0;
  graphnav::NavigationGraph::EdgeProperties edge_properties;
  edge_properties.trav_properties["default"].cost = 1.0;
  // far from obstacle or unknown, should create edge
  auto node_free_a = graph.add_node(Eigen::Vector3d(5.0, 0.0, 0.0), init_properties);
  auto node_free_b = graph.add_node(Eigen::Vector3d(0.0, 0.0, 0.0), init_properties);
  // across obstacle, should remove edge
  auto node_obs_a = graph.add_node(Eigen::Vector3d(0.0, 8.0, 0.0), init_properties);
  auto node_obs_b = graph.add_node(Eigen::Vector3d(0.0, 13.0, 0.0), init_properties);
  graph.add_edge(node_obs_a, node_obs_b, edge_properties);
  // too close to unknown, should not create edge
  auto node_unknown_a = graph.add_node(Eigen::Vector3d(19.5, 0.0, 0.0), init_properties);
  auto node_unknown_b = graph.add_node(Eigen::Vector3d(19.5, 2.0, 0.0), init_properties);
  // close/in unknown, should preserve edge
  auto node_unknown_keep_a = graph.add_node(Eigen::Vector3d(25, 0.0, 0.0), init_properties);
  auto node_unknown_keep_b = graph.add_node(Eigen::Vector3d(25, 2.0, 0.0), init_properties);
  graph.add_edge(node_unknown_keep_a, node_unknown_keep_b, edge_properties);
  // close to unknown, but across obstacle, should remove edge
  auto node_unknown_obs_a = graph.add_node(Eigen::Vector3d(19.5, 8.0, 0.0), init_properties);
  auto node_unknown_obs_b = graph.add_node(Eigen::Vector3d(19.5, 13.0, 0.0), init_properties);
  graph.add_edge(node_unknown_obs_a, node_unknown_obs_b, edge_properties);

  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update_nb_nodes_sample_ = 0;  // don't sample new nodes
  mapper_->traversability_update(msg, dbg_map_);

  EXPECT_TRUE(graph.edge_exists(node_free_a, node_free_b));
  EXPECT_FALSE(graph.edge_exists(node_obs_a, node_obs_b));
  EXPECT_FALSE(graph.edge_exists(node_unknown_a, node_unknown_b));
  EXPECT_TRUE(graph.edge_exists(node_unknown_keep_a, node_unknown_keep_b));
  EXPECT_FALSE(graph.edge_exists(node_unknown_obs_a, node_unknown_obs_b));

  // ASSERT_TRUE(false);
}

TEST_F(MapperTest, TraversabilityUpdateCreatesFrontiers)
{
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    grid_map_.getPosition(*it, position);
    if (position.x() > 20)
    {
      grid_map_.at("unknown", *it) = 1.0;
    }
    if (position.y() > 10)
    {
      grid_map_.at("traversability", *it) = 1.0;
      if (position.y() > 10.2)
      {  // unknown behind obstacle
        grid_map_.at("unknown", *it) = 1.0;
      }
    }
  }

  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->frontier_cell_max_dist_ = 20.0;  // large enough to get all frontiers TODO a bit hacky
  mapper_->traversability_update(msg, dbg_map_);

  double coll_radius = mapper_->traversability_classes_["default"].radius;
  double map_y_min = grid_map_.getPosition().y() - grid_map_.getLength().y() / 2;
  double map_x_min = grid_map_.getPosition().x() - grid_map_.getLength().x() / 2;
  std::vector<Eigen::Vector2d> frontiers;
  std::vector<bool> frontier_found;
  auto& graph = mapper_->get_graph();
  for (const auto& node : graph.get_all_nodes())
  {
    auto node_properties = graph.get_node_properties(node);
    for (const auto& frontier : node_properties.trav_properties["default"].frontier_points)
    {
      frontiers.push_back(frontier.head<2>());
    }
  }
  frontier_found.resize(frontiers.size(), false);
  double tol = 0.5;

  // frontiers along unknown
  // x = 20, y in [map_y_min, 10 - radius]
  for (double y = map_y_min; y < 10 - coll_radius; y += 0.1)
  {
    auto pos = Eigen::Vector2d(20, y);
    bool found_match = false;
    for (size_t i = 0; i < frontiers.size(); i++)
    {
      if ((frontiers[i] - pos).norm() < tol)
      {
        frontier_found[i] = true;
        found_match = true;
      }
    }
    ASSERT_TRUE(found_match) << "No frontier found at position " << pos.transpose();
  }

  // frontiers along map boundaries
  // x in [map_x_min, 20], y = map_y_min
  // x = map_x_min, y in [map_y_min, 10 - radius]
  for (double x = map_x_min; x < 20; x += 0.1)
  {
    auto pos = Eigen::Vector2d(x, map_y_min);
    bool found_match = false;
    for (size_t i = 0; i < frontiers.size(); i++)
    {
      if ((frontiers[i] - pos).norm() < tol)
      {
        frontier_found[i] = true;
        found_match = true;
      }
    }
    ASSERT_TRUE(found_match) << "No frontier found at position " << pos.transpose();
  }
  for (double y = map_y_min; y < 10 - coll_radius; y += 0.1)
  {
    auto pos = Eigen::Vector2d(map_x_min, y);
    bool found_match = false;
    for (size_t i = 0; i < frontiers.size(); i++)
    {
      if ((frontiers[i] - pos).norm() < tol)
      {
        frontier_found[i] = true;
        found_match = true;
      }
    }
    ASSERT_TRUE(found_match) << "No frontier found at position " << pos.transpose();
  }

  // should have no frontiers along obstacle
  // x in [map_x_min, 20], y = 10
  for (double x = map_x_min; x < 20; x += 0.1)
  {
    auto pos = Eigen::Vector2d(x, 10);
    ASSERT_TRUE(std::find_if(frontiers.begin(), frontiers.end(),
                             [&](const Eigen::Vector2d& frontier) { return (frontier - pos).norm() < tol; }) ==
                frontiers.end())
        << "Frontier found at position " << pos.transpose();
  }

  for (size_t i = 0; i < frontiers.size(); i++)
  {
    ASSERT_TRUE(frontier_found[i]) << "Frontier at position " << frontiers[i].transpose()
                                   << " was not found in expected locations";
  }

  // ASSERT_TRUE(false);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  if (std::find_if(argv, argv + argc, [](const char* arg) { return std::string(arg) == "--publish-on-failure"; }) !=
      argv + argc)
  {
    publish_on_failure_ = true;
  }
  return RUN_ALL_TESTS();
}

TEST_F(MapperTest, PhysicalEvidenceAgeAndTraversalSurviveGraphPublication)
{
  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties properties;
  properties.trav_properties["default"].visited = true;
  properties.trav_properties["default"].free_radius = 2.;
  properties.physical_observed_at_ns = 4000000000ULL;
  properties.metadata["physical_confidence"] = {.8f};
  auto a = graph.add_node(Eigen::Vector3d(1., 2., 0.), properties);
  auto b = graph.add_node(Eigen::Vector3d(5., 2., 0.), properties);
  graphnav::NavigationGraph::EdgeProperties connection;
  connection.trav_properties["default"].cost = 4.;
  connection.trav_properties["default"].traversed = true;
  connection.physical_observed_at_ns = 6000000000ULL;
  connection.metadata["physical_feasible"] = {0.f};
  connection.metadata["physical_confidence"] = {.8f};
  graph.add_edge(a, b, connection);
  for (const int seconds : {10, 20})
  {
    const auto message = graph.to_msg(rclcpp::Time(seconds, 0, RCL_ROS_TIME), a);
    ASSERT_EQ(message.nodes.size(), 2u);
    for (const auto& node : message.nodes)
    {
      ASSERT_EQ(node.robot_traversability.size(), 1u);
      EXPECT_EQ(node.robot_traversability[0].observed_at.sec, 4);
    }
    ASSERT_EQ(message.edges.size(), 1u);
    const auto& edge = message.edges[0];
    ASSERT_EQ(edge.robot_traversability.size(), 1u);
    EXPECT_EQ(edge.robot_traversability[0].observed_at.sec, 6);
    EXPECT_FLOAT_EQ(edge.robot_traversability[0].physical_traversability, 0.f);
    EXPECT_FLOAT_EQ(edge.robot_traversability[0].confidence, .8f);
    bool traversed = false;
    for (const auto& property : edge.traversability[0].properties)
      if (property.key == "traversed") traversed = property.value[0] > .5f;
    EXPECT_TRUE(traversed);
  }
}


TEST_F(MapperTest, OdometryJumpDoesNotCreateTraversedConnection)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  odom->pose.pose.orientation.w = 1.;
  mapper_->odometry_update(odom);
  odom->header.stamp = rclcpp::Time(1, 100000000, RCL_ROS_TIME);
  odom->pose.pose.position.x = 20.;
  mapper_->odometry_update(odom);
  const auto message = mapper_->get_graph_msg(rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_EQ(message.nodes.size(), 2u);
  EXPECT_TRUE(message.edges.empty());
}


TEST_F(MapperTest, OccupancyPlaneDoesNotClaimObservedTerrain)
{
  mapper_->traversability_update_nb_nodes_sample_ = 0;
  grid_map_.add("elevation_valid", 0.f);
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->header.stamp = stamp_;
  odom->pose.pose.position.x = 5.;
  odom->pose.pose.position.y = 2.;
  odom->pose.pose.orientation.w = 1.;
  mapper_->odometry_update(odom);
  grid_map_msgs::msg::GridMap::SharedPtr grid = grid_map::GridMapRosConverter::toMessage(grid_map_);
  mapper_->traversability_update(grid);
  const auto message = mapper_->get_graph_msg(rclcpp::Time(10, 0, RCL_ROS_TIME));
  ASSERT_EQ(message.nodes.size(), 1u);
  bool terrain_unknown = false;
  for (const auto& property : message.nodes[0].properties)
    if (property.key == "terrain_observed") terrain_unknown = property.value[0] == 0.f;
  EXPECT_TRUE(terrain_unknown);
  ASSERT_EQ(message.nodes[0].robot_traversability.size(), 1u);
  EXPECT_EQ(message.nodes[0].robot_traversability[0].observed_at.sec, stamp_.seconds());
}

TEST_F(MapperTest, MissingTraversabilityHasNoFreeSpaceClearance)
{
  mapper_->traversability_update_nb_nodes_sample_ = 0;
  grid_map_.get("traversability").setConstant(std::numeric_limits<float>::quiet_NaN());
  grid_map_msgs::msg::GridMap::SharedPtr msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  dbg_map_ = std::make_shared<grid_map_msgs::msg::GridMap>();
  mapper_->traversability_update(msg, dbg_map_);
  grid_map::GridMap result;
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(*dbg_map_, result));
  ASSERT_TRUE(result.exists("traversability_sdf"));
  EXPECT_TRUE(result.get("traversability_sdf").allFinite());
  EXPECT_FLOAT_EQ(result.atPosition("traversability_sdf", grid_map::Position(5., 2.)), 0.f);
}

TEST_F(MapperTest, LocalMapUsesTransformAtAcquisitionTime)
{
  mapper_->traversability_update_nb_nodes_sample_ = 0;
  grid_map_.setFrameId("local_map");
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "local_map";
  transform.transform.rotation.w = 1.;
  transform.header.stamp = stamp_;
  ASSERT_TRUE(tf_buffer_->setTransform(transform, "offline_test", false));
  transform.header.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  transform.transform.translation.x = 10.;
  ASSERT_TRUE(tf_buffer_->setTransform(transform, "offline_test", false));
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position p;
    grid_map_.getPosition(*it, p);
    if (std::abs(p.x()-5.) < 1. && std::abs(p.y()-2.) < 1.)
      grid_map_.at("traversability", *it) = 1.f;
  }
  graphnav::NavigationGraph::NodeProperties properties;
  properties.trav_properties["default"].free_radius = 2.;
  const auto node = mapper_->get_graph().add_node(Eigen::Vector3d(5., 2., 0.), properties);
  mapper_->traversability_update(grid_map::GridMapRosConverter::toMessage(grid_map_));
  EXPECT_FALSE(mapper_->get_graph().node_exists(node));
}

TEST_F(MapperTest, DuplicateOdometryCannotCreateTraversedConnection)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->header.stamp = stamp_;
  odom->pose.pose.orientation.w = 1.;
  mapper_->odometry_update(odom);
  odom->pose.pose.position.x = 1.5;
  mapper_->odometry_update(odom);
  const auto message = mapper_->get_graph_msg(rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_EQ(message.nodes.size(), 1u);
  EXPECT_TRUE(message.edges.empty());
}

TEST_F(MapperTest, RealHeightRampKeepsConnectionButCurbBlocksPhysicalTraversal)
{
  mapper_->traversability_update_nb_nodes_sample_ = 0;
  grid_map_.add("elevation_valid", 1.f);
  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties properties;
  properties.trav_properties["default"].free_radius = 2.;
  const auto a=graph.add_node(Eigen::Vector3d(2., 2., 0.),properties);
  const auto b=graph.add_node(Eigen::Vector3d(6., 2., 0.),properties);
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    grid_map::Position p; grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=.2*p.x();
  }
  mapper_->traversability_update(grid_map::GridMapRosConverter::toMessage(grid_map_));
  ASSERT_TRUE(graph.edge_exists(a,b));
  auto edge=graph.get_edge_properties(a,b);
  EXPECT_FLOAT_EQ(edge.metadata["physical_feasible"][0],1.f);
  EXPECT_NEAR(edge.metadata["maximum_slope"][0],.2f,1e-4);
  EXPECT_LT(edge.metadata["roughness"][0],1e-5);
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    grid_map::Position p; grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=p.x()>4.?.4f:0.f;
  }
  mapper_->traversability_update(grid_map::GridMapRosConverter::toMessage(grid_map_));
  ASSERT_TRUE(graph.edge_exists(a,b));
  edge=graph.get_edge_properties(a,b);
  EXPECT_FLOAT_EQ(edge.metadata["physical_feasible"][0],0.f);
}

TEST_F(MapperTest, WitnessedSubcellStepBlocksEdgeAndNode)
{
  const auto [a,b]=measured_connection();
  grid_map_.add("witnessed_step", 0.0);
  auto& graph=mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties properties;
  properties.trav_properties["default"].free_radius=2.;
  const auto c=graph.add_node(Eigen::Vector3d(4.,2.,0.),properties);
  for (grid_map::GridMapIterator it(grid_map_);!it.isPastEnd();++it) {
    grid_map::Position p;grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=0.f;
    if (p.x()>3.9 && p.x()<4.1 && std::abs(p.y()-2.)<.3)
      grid_map_.at("witnessed_step",*it)=.17f;
  }
  update_measured_map();
  const auto edge=mapper_->get_graph().get_edge_properties(a,b);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_feasible"),0.f);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_violation_observed"),1.f);
  EXPECT_GE(evidence(edge,"maximum_step_height"),.16f);
  const auto node=mapper_->get_graph().get_node_properties(c);
  EXPECT_FLOAT_EQ(evidence(node,"physical_feasible"),0.f);
  EXPECT_FLOAT_EQ(evidence(node,"physical_violation_observed"),1.f);
  EXPECT_GE(evidence(node,"step_height"),.16f);
}

TEST_F(MapperTest, MissingHeightKeepsExplorationTopologyWithoutInventingPhysicalConfidence)
{
  mapper_->traversability_update_nb_nodes_sample_ = 0;
  grid_map_.add("elevation_valid", 0.f);
  auto& graph = mapper_->get_graph();
  graphnav::NavigationGraph::NodeProperties properties;
  properties.trav_properties["default"].free_radius = 2.;
  const auto a=graph.add_node(Eigen::Vector3d(2.,2.,0.),properties);
  const auto b=graph.add_node(Eigen::Vector3d(6.,2.,0.),properties);
  mapper_->traversability_update(grid_map::GridMapRosConverter::toMessage(grid_map_));
  ASSERT_TRUE(graph.edge_exists(a,b));
  EXPECT_FLOAT_EQ(graph.get_node_properties(a).metadata.at("physical_confidence")[0],0.f);
  EXPECT_FLOAT_EQ(graph.get_node_properties(a).metadata.at("terrain_observed")[0],0.f);
  EXPECT_FLOAT_EQ(graph.get_edge_properties(a,b).metadata.at("physical_confidence")[0],0.f);
  EXPECT_FLOAT_EQ(graph.get_edge_properties(a,b).metadata.at("terrain_observed")[0],0.f);
}

TEST_F(MapperTest, MeasuredSteepSegmentCannotBeCancelledByMissingHeightElsewhere)
{
  const auto [a,b]=measured_connection();
  for (grid_map::GridMapIterator it(grid_map_);!it.isPastEnd();++it) {
    grid_map::Position p;grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=.6*std::clamp(p.x()-3.,0.,.4);
    if (p.x()>5. && p.x()<5.3) grid_map_.at("elevation_valid",*it)=0.f;
    if (p.x()>5. && p.x()<5.5 && std::abs(p.y()-2.)<.5) grid_map_.at("unknown",*it)=1.f;
  }
  update_measured_map();
  const auto edge=mapper_->get_graph().get_edge_properties(a,b);
  EXPECT_GT(evidence(edge,"maximum_slope"),.35f);
  EXPECT_FLOAT_EQ(evidence(edge,"terrain_observed"),0.f);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_feasible"),0.f);
  EXPECT_GE(evidence(edge,"physical_confidence"),.7f);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_violation_observed"),1.f);
  EXPECT_FLOAT_EQ(evidence(edge,"clearance_confidence"),0.f);
  const auto message=mapper_->get_graph().to_msg(stamp_,a);
  const auto serialized=std::find_if(message.edges.begin(),message.edges.end(),[&](const auto& item) {
    return message.nodes[item.from_idx].uuid==a.to_msg() && message.nodes[item.to_idx].uuid==b.to_msg();
  });
  ASSERT_NE(serialized,message.edges.end());
  EXPECT_FLOAT_EQ(serialized->robot_traversability[0].physical_traversability,0.f);
  EXPECT_GE(serialized->robot_traversability[0].confidence,.7f);
}

TEST_F(MapperTest, MissingHeightDoesNotJoinSeparateFlatSurfacesIntoRoughTerrain)
{
  const auto [a,b]=measured_connection();
  for (grid_map::GridMapIterator it(grid_map_);!it.isPastEnd();++it) {
    grid_map::Position p;grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=p.x()<4.?0.f:.6f;
    if (p.x()>3. && p.x()<5.) grid_map_.at("elevation_valid",*it)=0.f;
  }
  update_measured_map();
  const auto edge=mapper_->get_graph().get_edge_properties(a,b);
  EXPECT_NEAR(evidence(edge,"maximum_slope"),0.f,1e-6);
  EXPECT_NEAR(evidence(edge,"maximum_step_height"),0.f,1e-6);
  EXPECT_NEAR(evidence(edge,"roughness"),0.f,1e-6);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_confidence"),0.f);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_violation_observed"),0.f);
}

TEST_F(MapperTest, KnownNodeSlopeSurvivesUnknownSpaceNearItsFootprint)
{
  const auto [a,b]=measured_connection();(void)b;
  for (grid_map::GridMapIterator it(grid_map_);!it.isPastEnd();++it) {
    grid_map::Position p;grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=.6*p.x();
    if (p.x()>2.15 && p.x()<2.75 && std::abs(p.y()-2.)<.4) grid_map_.at("unknown",*it)=1.f;
  }
  update_measured_map();
  const auto node=mapper_->get_graph().get_node_properties(a);
  EXPECT_FLOAT_EQ(evidence(node,"physical_feasible"),0.f);
  EXPECT_GE(evidence(node,"physical_confidence"),.7f);
  EXPECT_FLOAT_EQ(evidence(node,"physical_violation_observed"),1.f);
}

TEST_F(MapperTest, UnknownUpdateDoesNotRefreshOrErasePreviousMeasuredViolation)
{
  const auto [a,b]=measured_connection();
  for (grid_map::GridMapIterator it(grid_map_);!it.isPastEnd();++it) {
    grid_map::Position p;grid_map_.getPosition(*it,p);
    grid_map_.at("elevation",*it)=.6*p.x();
  }
  update_measured_map();
  grid_map_["elevation_valid"].setZero();
  update_measured_map(2);
  const auto node=mapper_->get_graph().get_node_properties(a);
  const auto edge=mapper_->get_graph().get_edge_properties(a,b);
  EXPECT_FLOAT_EQ(evidence(node,"physical_violation_observed"),1.f);
  EXPECT_FLOAT_EQ(evidence(edge,"physical_violation_observed"),1.f);
  EXPECT_EQ(node.physical_observed_at_ns,1000000000ULL);
  EXPECT_EQ(edge.physical_observed_at_ns,1000000000ULL);
  grid_map_["elevation_valid"].setOnes();grid_map_["elevation"].setZero();
  update_measured_map(3);
  const auto cleared_node=mapper_->get_graph().get_node_properties(a);
  const auto cleared_edge=mapper_->get_graph().get_edge_properties(a,b);
  EXPECT_FLOAT_EQ(evidence(cleared_node,"physical_violation_observed"),0.f);
  EXPECT_FLOAT_EQ(evidence(cleared_edge,"physical_violation_observed"),0.f);
  EXPECT_FLOAT_EQ(evidence(cleared_node,"physical_feasible"),1.f);
  EXPECT_FLOAT_EQ(evidence(cleared_edge,"physical_feasible"),1.f);
  EXPECT_EQ(cleared_node.physical_observed_at_ns,3000000000ULL);
  EXPECT_EQ(cleared_edge.physical_observed_at_ns,3000000000ULL);
}

