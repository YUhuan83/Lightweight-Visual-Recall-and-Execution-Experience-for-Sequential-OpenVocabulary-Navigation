// SPDX-License-Identifier: Apache-2.0
// Actual plugin lifecycle and archived geometry; no vehicle or action clients.
#include <gtest/gtest.h>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include "wildos_ackermann_smac/planner.hpp"
#include "wildos_ackermann_smac/path_contract.hpp"
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

class NativeRawAudit : public nav2_smac_planner::SmacPlannerHybrid {
public: void rawOnly() { _smoother.reset(); }
};
TEST_F(ActualPlanner, SealedCandidateBudgetAudit) {
 std::ifstream in("/home/ly/planner_wait_audit_20261015/input.txt"); ASSERT_TRUE(in.good());
 unsigned w,h;double res,ox,oy;Pose actual,goal;
 in>>w>>h>>res>>ox>>oy>>actual.x>>actual.y>>actual.yaw>>goal.x>>goal.y>>goal.yaw;
 ASSERT_TRUE(in.good());grid->resizeMap(w,h,res,ox,oy);
 for(unsigned i=0;i<w*h;i++){int v;in>>v;ASSERT_FALSE(in.fail()); unsigned char c=v<0?255:v==100?254:v==99?253:v==0?0:static_cast<unsigned char>(1+std::round((v-1)*251./97.));grid->setCost(i%w,i/w,c);}
 ASSERT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",12.}).successful);
 configure();bool production_valid=false;auto began=Clock::now();
 try {auto path=planner->createPlan(stamped(actual),stamped(goal)); production_valid=!path.poses.empty();}
 catch(const std::exception& e){std::cout<<"AUDIT production_error="<<e.what()<<std::endl;}
 std::cout<<"AUDIT production_valid="<<production_valid<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;
 planner->deactivate();planner->cleanup();planner.reset();
 Limits lim;lim.radius=12.;auto result=search(*grid,map->getRobotFootprint(),actual,lim,Clock::now()+std::chrono::milliseconds(200));
 std::cout<<"AUDIT connectors="<<result.curves.size()<<std::endl;
 NativeRawAudit native;native.configure(node,"CarLike",tf,map);native.rawOnly();native.activate();
 unsigned rank=0;
 for(const auto& curve:result.curves){
  auto anchor=curve.poses.back();bool valid=false;size_t count=0;std::string error;began=Clock::now();
  try {
   auto suffix=native.createPlan(stamped(anchor),stamped(goal));count=suffix.poses.size();std::vector<Pose> combined=curve.poses;
   valid=!suffix.poses.empty();
   for(size_t i=0;i<suffix.poses.size();i++){
    auto pose=suffix.poses[i].pose;Pose p{pose.position.x,pose.position.y,tf2::getYaw(pose.orientation)};
    if(i)combined.push_back(p);
    if(!safe(*grid,map->getRobotFootprint(),p,true))valid=false;
   }
   valid=valid && curvatureFeasible(combined,res,12.,72);
  }catch(const std::exception& e){error=e.what();}
  std::cout<<"AUDIT candidate="<<rank++<<" raw_poses="<<count<<" valid="<<valid<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<" error="<<error<<std::endl;
 }
 native.deactivate();native.cleanup();
}
