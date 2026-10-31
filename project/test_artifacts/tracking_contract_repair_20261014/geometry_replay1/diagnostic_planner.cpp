// SPDX-License-Identifier: Apache-2.0
// Actual plugin lifecycle and archived geometry; no vehicle or action clients.
#include <gtest/gtest.h>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include "nav2_smac_planner/smoother.hpp"
#include "wildos_ackermann_smac/planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace wildos_ackermann_smac;
class ActualPlanner : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("start_contract_probe");
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("start_probe_map");
    map->set_parameters({{"plugins",std::vector<std::string>{"inflation_layer"}},
      {"global_frame","world"},{"robot_base_frame","base"},
      {"resolution",.25},{"width",80},{"height",80},{"origin_x",-40.},{"origin_y",-40.},
      {"track_unknown_space",false},{"footprint_padding",0.},
      {"footprint","[[3.41,1.06],[3.41,-1.06],[-1.06,-1.06],[-1.06,1.06]]"}});
    map->declare_parameter("inflation_layer.plugin","nav2_costmap_2d::InflationLayer");
    map->declare_parameter("inflation_layer.inflation_radius",3.75);
    map->declare_parameter("inflation_layer.cost_scaling_factor",2.);
    map->configure(); ASSERT_FALSE(map->getUseRadius());
    grid=map->getCostmap(); grid->resizeMap(320,320,.25,-40.,-40.);
    std::fill(grid->getCharMap(),grid->getCharMap()+320*320,0);
    tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    for(const auto & p:std::vector<rclcpp::Parameter>{
      {"CarLike.motion_model_for_search","REEDS_SHEPP"},{"CarLike.minimum_turning_radius",8.},
      {"CarLike.angle_quantization_bins",72},{"CarLike.allow_unknown",true},
      {"CarLike.tolerance",1.},{"CarLike.max_iterations",300000},
      {"CarLike.max_on_approach_iterations",1000},{"CarLike.max_planning_time",1.5},
      {"CarLike.analytic_expansion_ratio",3.5},{"CarLike.analytic_expansion_max_length",50.},
      {"CarLike.non_straight_penalty",1.2},{"CarLike.cost_penalty",2.},
      {"CarLike.lookup_table_size",20.}}) { node->declare_parameter(p.get_name(),p.get_parameter_value()); }
    loader=std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>("nav2_core","nav2_core::GlobalPlanner");
  }
  void configure() {
    planner=loader->createSharedInstance("wildos_ackermann_smac/Planner");
    planner->configure(node,"CarLike",tf,map); planner->activate();
  }
  void TearDown() override {
    if(planner) { planner->deactivate();planner->cleanup();planner.reset(); }
    loader.reset();map->cleanup();map.reset();tf.reset();node.reset();
  }
  geometry_msgs::msg::PoseStamped stamped(Pose p) {
    geometry_msgs::msg::PoseStamped result;result.header.frame_id="world";
    result.pose.position.x=p.x;result.pose.position.y=p.y;
    tf2::Quaternion q;q.setRPY(0,0,p.yaw);result.pose.orientation=tf2::toMsg(q);return result;
  }
  void inflate() {
    auto layer=std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(map->getLayeredCostmap()->getPlugins()->at(0));
    layer->matchSize();layer->updateCosts(*grid,0,0,grid->getSizeInCellsX(),grid->getSizeInCellsY());
  }
  Pose loadArchived() {
    std::ifstream in(std::string(FIXTURE_DIR)+"/live_global_snapshot.txt");
    if(!in.good()) throw std::runtime_error("Archived map missing");
    unsigned w,h;double res,ox,oy;Pose actual,first,goal;
    in>>w>>h>>res>>ox>>oy>>actual.x>>actual.y>>actual.yaw>>first.x>>first.y>>first.yaw>>goal.x>>goal.y>>goal.yaw;
    if(!in.good()) throw std::runtime_error("Invalid archived map");
    grid->resizeMap(w,h,res,ox,oy);
    std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
    unsigned i;int cost;while(in>>i>>cost) {
      if(i>=w*h) throw std::runtime_error("Invalid archived cell");
      if(cost==100 || cost<0) grid->setCost(i%w,i/w,cost==100?254:255);
    }
    inflate();return actual;
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  nav2_costmap_2d::Costmap2D * grid;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader;
  std::shared_ptr<nav2_core::GlobalPlanner> planner;
};
TEST_F(ActualPlanner, OrdinaryPathAndFrozenParameters) {
  configure();auto p=planner->createPlan(stamped({.125,.125,0}),stamped({5.125,.125,0}));
  ASSERT_GT(p.poses.size(),2u); EXPECT_EQ(p.header.frame_id,"world");
  EXPECT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",8.}).successful);
  EXPECT_FALSE(node->set_parameter({"CarLike.minimum_turning_radius",6.}).successful);
  EXPECT_FALSE(node->set_parameter({"CarLike.max_planning_time",3.}).successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("CarLike.minimum_turning_radius").as_double(),8.);
}
TEST_F(ActualPlanner, CollisionRejectAndRecovery) {
  unsigned x,y;grid->worldToMap(2,0,x,y);grid->setCost(x,y,254);inflate();configure();
  EXPECT_THROW(planner->createPlan(stamped({.125,.125,0}),stamped({5.125,.125,0})),nav2_core::PlannerException);
  std::fill(grid->getCharMap(),grid->getCharMap()+320*320,0);
  EXPECT_GT(planner->createPlan(stamped({.125,.125,0}),stamped({5.125,.125,0})).poses.size(),2u);
  auto start=stamped({.125,.125,0});start.header.frame_id="wrong";
  EXPECT_THROW(planner->createPlan(start,stamped({5.125,.125,0})),nav2_core::PlannerException);
  grid->resizeMap(160,160,.5,-40.,-40.);
  EXPECT_THROW(planner->createPlan(stamped({.125,.125,0}),stamped({5.125,.125,0})),nav2_core::PlannerException);
}
TEST_F(ActualPlanner, InactiveChangesRequireReconfigureAndInvalidGoalsRejected) {
  configure();auto start=stamped({.125,.125,0}),goal=stamped({5.125,.125,0});
  goal.pose.position.x=NAN;
  EXPECT_THROW(planner->createPlan(start,goal),nav2_core::PlannerException);
  goal=stamped({5.125,.125,0});goal.pose.orientation.w=0;
  EXPECT_THROW(planner->createPlan(start,goal),nav2_core::PlannerException);
  planner->deactivate();
  EXPECT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",6.}).successful);
  EXPECT_THROW(planner->activate(),std::runtime_error);
}
TEST_F(ActualPlanner, ArchivedContinuousStartAutomaticallyConnected) {
  Pose actual=loadArchived();configure();
  nav2_smac_planner::SmacPlannerHybrid native;native.configure(node,"CarLike",tf,map);native.activate();
  auto failed=native.createPlan(stamped(actual),stamped({725.375,-47.875,-1.134464}));
  EXPECT_TRUE(failed.poses.empty());native.deactivate();native.cleanup();
  const auto began=Clock::now();
  auto path=planner->createPlan(stamped(actual),stamped({725.375,-47.875,-1.134464}));
  EXPECT_LT(std::chrono::duration<double>(Clock::now()-began).count(),2.);
  ASSERT_GT(path.poses.size(),2u);
  std::ofstream csv("archived_connected_path.csv");
  csv<<std::setprecision(17)<<"x,y,yaw\n";
  EXPECT_NEAR(path.poses.front().pose.position.x,actual.x,1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y,actual.y,1e-9);
  for(const auto & p:path.poses) {
    csv<<p.pose.position.x<<","<<p.pose.position.y<<","<<tf2::getYaw(p.pose.orientation)<<"\n";
    EXPECT_TRUE(safe(*grid,map->getRobotFootprint(),
      {p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)},true));
  }
}
TEST_F(ActualPlanner, ExhaustedConnectorRejectsThenFreeMapRecovers) {
  Pose actual=loadArchived();
  node->declare_parameter("CarLike.connector_max_evaluations",1);
  configure();
  auto goal=stamped({725.375,-47.875,-1.134464});
  EXPECT_THROW(planner->createPlan(stamped(actual),goal),nav2_core::PlannerException);
  std::fill(grid->getCharMap(),grid->getCharMap()+grid->getSizeInCellsX()*grid->getSizeInCellsY(),0);
  // Clearing obstacles does not remove the off-lattice kinematic mismatch.
  // An exhausted connector must not silently return a snapped start pose.
  EXPECT_THROW(planner->createPlan(stamped(actual),goal),nav2_core::PlannerException);
  planner->deactivate();planner->cleanup();planner.reset();
  ASSERT_TRUE(node->set_parameter({"CarLike.connector_max_evaluations",6000}).successful);
  configure();
  auto recovered=planner->createPlan(stamped(actual),goal);
  ASSERT_GT(recovered.poses.size(),2u);
  EXPECT_NEAR(recovered.poses.front().pose.position.x,actual.x,1e-9);
  EXPECT_NEAR(recovered.poses.front().pose.position.y,actual.y,1e-9);
  EXPECT_NEAR(std::remainder(tf2::getYaw(recovered.poses.front().pose.orientation)-actual.yaw,
    2.*std::acos(-1.)),0.,1e-9);
}
TEST_F(ActualPlanner, InvalidBudgetCannotConfigure) {
  node->declare_parameter("CarLike.max_total_planning_time",1.5);
  planner=loader->createSharedInstance("wildos_ackermann_smac/Planner");
  EXPECT_THROW(planner->configure(node,"CarLike",tf,map),std::runtime_error);
}

TEST_F(ActualPlanner, FreeOffLatticeStartPreservesActualPoseAndHeading) {
  configure();
  std::cout<<"START_PARAMETER: smooth_path="<<node->get_parameter("CarLike.smooth_path").as_bool()<<std::endl;
  for(double angle:{0.,1.5707963267948966}) {
    auto transform=[&](Pose p) {return Pose{p.x*std::cos(angle)-p.y*std::sin(angle),
      p.x*std::sin(angle)+p.y*std::cos(angle),p.yaw+angle};};
    const Pose actual=transform({.17,.21,.045});
    auto path=planner->createPlan(stamped(actual),stamped(transform({6.125,4.125,.6981317007977318})));
    ASSERT_GT(path.poses.size(),2u);
    const auto & first=path.poses.front().pose;
    std::cout<<"START_CONTRACT: "<<std::setprecision(17)<<"requested=("<<actual.x<<","<<actual.y
      <<","<<actual.yaw<<") returned=("<<first.position.x<<","<<first.position.y<<","<<tf2::getYaw(first.orientation)<<")"<<std::endl;
    EXPECT_NEAR(first.position.x,actual.x,1e-9);
    EXPECT_NEAR(first.position.y,actual.y,1e-9);
    EXPECT_NEAR(std::remainder(tf2::getYaw(first.orientation)-actual.yaw,2.*std::acos(-1.)),0.,1e-9);
  }
}

TEST_F(ActualPlanner, RecordedTownGoalReplannedFromActualAxlePose) {
  // Same actual start and unchanged endpoint as the failed field section.
  // A free map isolates kinematics; it cannot establish field traversability.
  const Pose actual{728.1258963294214,-5.380221515252725,2.397677766000295};
  const Pose goal{712.0949879209489,-3.39056020561104,2.4434611797323043};
  grid->resizeMap(320,320,.25,690.,-40.);
  std::fill(grid->getCharMap(),grid->getCharMap()+320*320,0);
  if(const auto setting=std::getenv("WILDOS_AUDIT_PLAN_RADIUS")) {
    const double radius=std::stod(setting);
    ASSERT_GE(radius,8.);ASSERT_TRUE(std::isfinite(radius));
    ASSERT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",radius}).successful);
  }
  node->declare_parameter("CarLike.smooth_path",!std::getenv("WILDOS_AUDIT_NO_SMOOTH"));
  configure();
  auto path=planner->createPlan(stamped(actual),stamped(goal));
  ASSERT_GT(path.poses.size(),2u);
  EXPECT_NEAR(path.poses.front().pose.position.x,actual.x,1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y,actual.y,1e-9);
  EXPECT_NEAR(std::remainder(tf2::getYaw(path.poses.front().pose.orientation)-actual.yaw,
    2.*std::acos(-1.)),0.,1e-9);
  EXPECT_LT(std::hypot(path.poses.back().pose.position.x-goal.x,
    path.poses.back().pose.position.y-goal.y),.2);
  const auto filename=std::getenv("WILDOS_AUDIT_PLAN_OUTPUT");
  std::ofstream csv(filename?filename:"recorded_town_replanned.csv");
  ASSERT_TRUE(csv.good());csv<<std::setprecision(17)<<"kind,x,y,yaw\n";
  csv<<"start,"<<actual.x<<","<<actual.y<<","<<actual.yaw<<"\n";
  for(const auto & p:path.poses) {
    csv<<"path,"<<p.pose.position.x<<","<<p.pose.position.y<<","<<tf2::getYaw(p.pose.orientation)<<"\n";
    EXPECT_TRUE(safe(*grid,map->getRobotFootprint(),
      {p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)},true));
  }
}

TEST_F(ActualPlanner, FullTrialRecordedGeometryAudit) {
  const int index=std::stoi(std::getenv("GEOMETRY_CASE"));
  const bool smoothing=std::getenv("GEOMETRY_SMOOTH");
  const bool free_map=std::getenv("GEOMETRY_FREE");
  const std::string filename="input"+std::to_string(index)+".txt";
  std::ifstream input(filename); ASSERT_TRUE(input.good());
  unsigned width,height;double res,ox,oy;Pose start,goal;
  input>>width>>height>>res>>ox>>oy>>start.x>>start.y>>start.yaw>>goal.x>>goal.y>>goal.yaw;
  ASSERT_TRUE(input.good()); grid->resizeMap(width,height,res,ox,oy);
  for(unsigned i=0;i<width*height;i++) {
    int value;input>>value;ASSERT_FALSE(input.fail());
    unsigned char cost=value<0?255:value==100?254:value==99?253:value==0?0:1+(value-1)*251/97;
    grid->setCost(i%width,i/width,free_map?0:cost);
  }
  ASSERT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",12.}).successful);
  node->declare_parameter("CarLike.smooth_path",smoothing);
  configure();
  const std::string label="case"+std::to_string(index)+(free_map?"_free":"_recorded")+(smoothing?"_smooth":"_raw");
  try {
    auto path=planner->createPlan(stamped(start),stamped(goal));
    ASSERT_GT(path.poses.size(),2u);
    std::ofstream output(label+".csv");ASSERT_TRUE(output.good());
    output<<std::setprecision(17)<<"x,y,yaw\n";
    for(const auto & p:path.poses) output<<p.pose.position.x<<","<<p.pose.position.y<<","<<tf2::getYaw(p.pose.orientation)<<"\n";
    std::cout<<"GEOMETRY_PLAN "<<label<<" poses="<<path.poses.size()<<std::endl;
  } catch(const std::runtime_error & e) {
    std::ofstream(label+".error")<<e.what()<<"\n";
    std::cout<<"GEOMETRY_PLAN "<<label<<" failed="<<e.what()<<std::endl;
  }
}
