// Public Nav2 plugin API tests: actual TF, costmap, controller and goal checker.
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {
struct ControllerFixture {
  pluginlib::ClassLoader<nav2_core::Controller> controllers{"nav2_core", "nav2_core::Controller"};
  pluginlib::ClassLoader<nav2_core::GoalChecker> checkers{"nav2_core", "nav2_core::GoalChecker"};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<nav2_core::Controller> controller;
  std::shared_ptr<nav2_core::GoalChecker> checker;
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  nav_msgs::msg::Path path;

  ControllerFixture() {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("rpp_blackbox");
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("rpp_blackbox_map");
    map->set_parameters({{"plugins",std::vector<std::string>{"inflation_layer"}},
      {"global_frame","spot1/odom"},{"robot_base_frame","spot1/nav_base_link"},
      {"width",80},{"height",80},{"resolution",.2},{"track_unknown_space",false},
      {"footprint_padding",0.},
      {"footprint","[[3.41,1.06],[3.41,-1.06],[-1.06,-1.06],[-1.06,1.06]]"}});
    map->declare_parameter("inflation_layer.plugin","nav2_costmap_2d::InflationLayer");
    map->declare_parameter("inflation_layer.inflation_radius",1.6);
    map->configure();
    auto grid=map->getCostmap();grid->resizeMap(400,400,.2,-40,-40);
    std::fill(grid->getCharMap(),grid->getCharMap()+400*400,0);
    for (const auto & p:std::vector<rclcpp::Parameter>{
      {"FollowPath.minimum_turning_radius",8.},{"FollowPath.continuous_path_execution",true},
      {"FollowPath.use_rotate_to_heading",false},{"FollowPath.allow_reversing",true},
      {"FollowPath.use_collision_detection",true},{"FollowPath.desired_linear_vel",1.2},
      {"FollowPath.lookahead_dist",3.},{"FollowPath.min_lookahead_dist",3.},
      {"FollowPath.max_lookahead_dist",6.},{"FollowPath.lookahead_time",2.5},
      {"FollowPath.use_velocity_scaled_lookahead_dist",false},
      {"FollowPath.max_robot_pose_search_dist",10.},
      {"FollowPath.min_approach_linear_velocity",.2},
      {"FollowPath.approach_velocity_scaling_dist",2.},
      {"FollowPath.regulated_linear_scaling_min_radius",8.},
      {"FollowPath.regulated_linear_scaling_min_speed",.3},
      {"goal_checker.xy_goal_tolerance",1.},{"goal_checker.yaw_goal_tolerance",.7}}) {
      node->declare_parameter(p.get_name(),p.get_parameter_value());
    }
    controller=controllers.createSharedInstance("wildos_ackermann_rpp/Controller");
    checker=checkers.createSharedInstance("wildos_ackermann_rpp/ContinuousGoalChecker");
    controller->configure(node,"FollowPath",map->getTfBuffer(),map);
    checker->initialize(node,"goal_checker",map);
    controller->activate();
    move(0,0,0);
    path.header=pose.header;
  }
  ~ControllerFixture() {
    controller->deactivate();controller->cleanup();checker.reset();controller.reset();
    map->cleanup();map.reset();node.reset();rclcpp::shutdown();
  }
  void move(double x,double y,double yaw) {
    pose.header.frame_id="spot1/odom";pose.header.stamp=node->now();
    pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.position.z=0.;
    tf2::Quaternion q;q.setRPY(0,0,yaw);pose.pose.orientation=tf2::toMsg(q);
    geometry_msgs::msg::TransformStamped transform;
    transform.header=pose.header;transform.child_frame_id="spot1/nav_base_link";
    transform.transform.translation.x=x;transform.transform.translation.y=y;
    transform.transform.rotation=pose.pose.orientation;
    map->getTfBuffer()->setTransform(transform,"blackbox",true);
  }
  void add(double x,double y=0.,double yaw=0.) {
    auto point=pose;point.pose.position.x=x;point.pose.position.y=y;
    tf2::Quaternion q;q.setRPY(0,0,yaw);point.pose.orientation=tf2::toMsg(q);
    path.poses.push_back(point);
  }
  geometry_msgs::msg::TwistStamped command() {
    return controller->computeVelocityCommands(pose,speed,checker.get());
  }
  void loadRecorded() {
    std::ifstream input(RECORDED_HANDOFF_FILE);
    unsigned width,height,count;double resolution,ox,oy,x,y,yaw;
    input>>width>>height>>resolution>>ox>>oy>>x>>y>>yaw;
    input>>speed.linear.x>>speed.linear.y>>speed.angular.z>>count;
    ASSERT_TRUE(input.good());move(x,y,yaw);path.header=pose.header;
    for(unsigned i=0;i<count;++i) {input>>x>>y>>yaw;add(x,y,yaw);}
    auto grid=map->getCostmap();grid->resizeMap(width,height,resolution,ox,oy);
    std::fill(grid->getCharMap(),grid->getCharMap()+width*height,0);
    unsigned index;int cost;
    while(input>>index>>cost) {
      ASSERT_LT(index,width*height);ASSERT_GE(cost,0);ASSERT_LE(cost,255);
      grid->setCost(index%width,index/width,cost);
    }
  }
};
}

TEST(ControllerFlow, ForwardAndReversePlansKeepTheirDirection) {
  ControllerFixture f;
  for(double x:{0.,1.,2.,4.,8.}) {f.add(x);}
  f.controller->setPlan(f.path);
  EXPECT_GT(f.command().twist.linear.x,0.);
  f.path.poses.clear();
  for(double x:{0.,-1.,-2.,-4.,-8.}) {f.add(x);}
  f.controller->setPlan(f.path);
  EXPECT_LT(f.command().twist.linear.x,0.);
}

TEST(ControllerFlow, RecordedMovingHandoffUsesRemainingSection) {
  ControllerFixture f;f.loadRecorded();
  f.controller->setPlan(f.path);
  geometry_msgs::msg::TwistStamped command;
  ASSERT_NO_THROW(command=f.command());
  EXPECT_DOUBLE_EQ(command.twist.linear.x,0.); // Actual car is still reversing.
  f.speed=geometry_msgs::msg::Twist();
  ASSERT_NO_THROW(command=f.command());
  EXPECT_GT(command.twist.linear.x,0.);
  EXPECT_LE(std::abs(command.twist.angular.z/command.twist.linear.x),1./8.+1e-9);
}

TEST(ControllerFlow, CuspBrakePersistsWhileCoastingPastPositionTolerance) {
  ControllerFixture f;
  for(double x:{0.,.25,.5,.75,1.,.75,.5,0.,-1.,-3.}) {f.add(x);}
  f.controller->setPlan(f.path);
  f.move(.85,0,0);f.speed.linear.x=.6;
  EXPECT_DOUBLE_EQ(f.command().twist.linear.x,0.);
  f.move(1.25,0,0);f.speed.linear.x=.2;
  geometry_msgs::msg::TwistStamped command;
  ASSERT_NO_THROW(command=f.command());
  EXPECT_DOUBLE_EQ(command.twist.linear.x,0.);
  f.speed=geometry_msgs::msg::Twist();
  EXPECT_LT(f.command().twist.linear.x,0.);
}
