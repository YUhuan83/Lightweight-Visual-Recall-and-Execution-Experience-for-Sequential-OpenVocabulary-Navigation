// SPDX-License-Identifier: Apache-2.0
// Actual plugin, static TF and in-memory costmap. No simulator or vehicle.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <fstream>
#include <map>
#include <sstream>
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/transform_listener.h"

class Goal : public nav2_core::GoalChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &,const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override { return false; }
  bool getTolerances(geometry_msgs::msg::Pose & pose,geometry_msgs::msg::Twist & velocity) override {
    pose.position.x=pose.position.y=.6;
    velocity.linear.x=velocity.angular.z=std::numeric_limits<double>::lowest();return true;
  }
};

class ActualPlugin : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("curvature_contract_probe");
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("curvature_probe_costmap");
    map->set_parameters({rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame","world"),rclcpp::Parameter("robot_base_frame","base"),
      rclcpp::Parameter("resolution",.025),rclcpp::Parameter("width",20),rclcpp::Parameter("height",20),
      rclcpp::Parameter("origin_x",-10.),rclcpp::Parameter("origin_y",-10.),
      rclcpp::Parameter("robot_radius",.05),rclcpp::Parameter("footprint_padding",0.)});
    map->configure();
    map->getCostmap()->resizeMap(800,800,.025,-10.,-10.);
    std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,
      nav2_costmap_2d::FREE_SPACE);
    map->setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(.05));
    tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener=std::make_unique<tf2_ros::TransformListener>(*tf,node,true);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="world";transform.child_frame_id="base";
    transform.transform.rotation.w=1.;tf->setTransform(transform,"test",true);
    for (const auto & p : std::vector<rclcpp::Parameter>{
        {"FollowPath.desired_linear_vel",1.},{"FollowPath.lookahead_dist",3.},
        {"FollowPath.use_rotate_to_heading",false},{"FollowPath.allow_reversing",true},
        {"FollowPath.use_collision_detection",true},
        {"FollowPath.use_velocity_scaled_lookahead_dist",false},
        {"FollowPath.use_regulated_linear_velocity_scaling",false},
        {"FollowPath.use_cost_regulated_linear_velocity_scaling",false},
        {"FollowPath.approach_velocity_scaling_dist",.1},
        {"FollowPath.max_allowed_time_to_collision_up_to_carrot",3.2},
        {"FollowPath.minimum_turning_radius",8.}}) {
      node->declare_parameter(p.get_name(),p.get_parameter_value());
    }
    loader=std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>("nav2_core","nav2_core::Controller");
    bounded=loader->createSharedInstance("wildos_ackermann_rpp/Controller");
    bounded->configure(node,"FollowPath",tf,map);bounded->activate();
    baseline=std::make_shared<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
    baseline->configure(node,"FollowPath",tf,map);baseline->activate();
    goal_loader=std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>("nav2_core","nav2_core::GoalChecker");
  }
  void TearDown() override {
    baseline->deactivate();baseline->cleanup();baseline.reset();
    bounded->deactivate();bounded->cleanup();bounded.reset();loader.reset();goal_loader.reset();
    map->cleanup();map.reset();listener.reset();tf.reset();node.reset();
  }
  void setPath(double sign) {
    nav_msgs::msg::Path path;path.header.frame_id="world";path.header.stamp=node->now();
    for (const auto & point : std::vector<std::pair<double,double>>{{0.,0.},{std::sqrt(8.)*sign,1.},{4.*sign,2.}}) {
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=point.first;pose.pose.position.y=point.second;pose.pose.orientation.w=1.;
      path.poses.push_back(pose);
    }
    bounded->setPlan(path);baseline->setPlan(path);
  }
  geometry_msgs::msg::TwistStamped command(std::shared_ptr<nav2_core::Controller> controller) {
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="world";
    pose.header.stamp=node->now();pose.pose.orientation.w=1.;
    geometry_msgs::msg::Twist speed;speed.linear.x=1.;
    return controller->computeVelocityCommands(pose,speed,&goal);
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader;
  std::shared_ptr<nav2_core::Controller> bounded,baseline;
  Goal goal;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> goal_loader;
  std::shared_ptr<nav2_core::GoalChecker> stoppedChecker() {
    for (const auto & p : std::vector<rclcpp::Parameter>{
        {"StopProbe.xy_goal_tolerance",.2},{"StopProbe.yaw_goal_tolerance",.35},
        {"StopProbe.stateful",false},{"StopProbe.trans_stopped_velocity",.05},
        {"StopProbe.rot_stopped_velocity",.05}}) {
      node->declare_parameter(p.get_name(),p.get_parameter_value());
    }
    auto checker=goal_loader->createSharedInstance("nav2_controller::StoppedGoalChecker");
    checker->initialize(node,"StopProbe",map);return checker;
  }
  nav_msgs::msg::Path straightPath(std::vector<double> positions,double final_yaw=0.) {
    nav_msgs::msg::Path path;path.header.frame_id="world";path.header.stamp=node->now();
    for (double x:positions) {
      geometry_msgs::msg::PoseStamped p;p.header=path.header;
      p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.push_back(p);
    }
    path.poses.back().pose.orientation.z=std::sin(final_yaw/2.);
    path.poses.back().pose.orientation.w=std::cos(final_yaw/2.);
    bounded->setPlan(path);baseline->setPlan(path);return path;
  }
  geometry_msgs::msg::TwistStamped commandWith(std::shared_ptr<nav2_core::Controller> c,nav2_core::GoalChecker * g) {
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="world";pose.header.stamp=node->now();
    pose.pose.orientation.w=1.;geometry_msgs::msg::Twist velocity;velocity.linear.x=.2;
    return c->computeVelocityCommands(pose,velocity,g);
  }
  geometry_msgs::msg::TwistStamped commandAt(std::shared_ptr<nav2_core::Controller> c,
    nav2_core::GoalChecker * g,double x) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="world";transform.child_frame_id="base";
    transform.transform.translation.x=x;transform.transform.rotation.w=1.;
    tf->setTransform(transform,"test",true);
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="world";
    pose.header.stamp=node->now();pose.pose.position.x=x;pose.pose.orientation.w=1.;
    geometry_msgs::msg::Twist velocity;velocity.linear.x=.2;
    return c->computeVelocityCommands(pose,velocity,g);
  }
};

TEST_F(ActualPlugin, BothDirectionsPublishTheArcTheyCanExecute) {
  for (double sign : {-1.,1.}) {
    setPath(sign);
    auto old=command(baseline);auto fixed=command(bounded);
    EXPECT_GT(std::abs(old.twist.angular.z/old.twist.linear.x),.125);
    EXPECT_DOUBLE_EQ(fixed.twist.linear.x,old.twist.linear.x);
    EXPECT_LE(std::abs(fixed.twist.angular.z/fixed.twist.linear.x),.125+1e-9);
    EXPECT_GT(sign*fixed.twist.linear.x,0.);
  }
}
TEST_F(ActualPlugin, CollisionIsCheckedOnTheBoundedArc) {
  for (double sign : {-1.,1.}) {
    std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,
      nav2_costmap_2d::FREE_SPACE);
    unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(sign*2.5,.39,x,y));
    map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
    setPath(sign);
    EXPECT_NO_THROW(command(baseline));
    EXPECT_THROW(command(bounded),nav2_core::PlannerException);
  }
}
TEST_F(ActualPlugin, RadiusIsFrozenAcrossDynamicParameterUpdates) {
  auto rejected=node->set_parameters({rclcpp::Parameter("FollowPath.minimum_turning_radius",4.)});
  ASSERT_EQ(rejected.size(),1u);EXPECT_FALSE(rejected[0].successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.minimum_turning_radius").as_double(),8.);
}

TEST_F(ActualPlugin, CuspBrakesInBothDirectionsAndRequiresMeasuredStop) {
  auto checker=stoppedChecker();geometry_msgs::msg::Pose pose;pose.orientation.w=1.;
  geometry_msgs::msg::Twist moving,stopped;moving.linear.x=.2;
  for (double sign:{-1.,1.}) {
    auto path=straightPath({0.,sign*.15});
    EXPECT_FALSE(checker->isGoalReached(pose,path.poses.back().pose,moving));
    EXPECT_TRUE(checker->isGoalReached(pose,path.poses.back().pose,stopped));
    auto old=commandWith(baseline,checker.get());EXPECT_GT(std::abs(old.twist.linear.x),0.);
    auto fixed=commandWith(bounded,checker.get());
    EXPECT_DOUBLE_EQ(fixed.twist.linear.x,0.);EXPECT_DOUBLE_EQ(fixed.twist.angular.z,0.);
    straightPath({0.,sign*.21});
    EXPECT_GT(sign*commandWith(bounded,checker.get()).twist.linear.x,0.);
  }
}

TEST_F(ActualPlugin, ALocalCropOrWrongHeadingCannotTriggerTerminalBraking) {
  auto checker=stoppedChecker();straightPath({0.,.15,30.});
  EXPECT_GT(commandWith(bounded,checker.get()).twist.linear.x,0.);
  straightPath({0.,.15},.5);
  EXPECT_GT(commandWith(bounded,checker.get()).twist.linear.x,0.);
}

TEST_F(ActualPlugin, TerminalBrakingDoesNotDisableFootprintCollisionCheck) {
  auto checker=stoppedChecker();
  for (double obstacle_x:{0.,.05}) {
    std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,
      nav2_costmap_2d::FREE_SPACE);
    straightPath({0.,.15});
    unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(obstacle_x,0.,x,y));
    map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_THROW(commandWith(bounded,checker.get()),nav2_core::PlannerException);
  }
}

TEST_F(ActualPlugin, AStationaryCommandDoesNotScanAHypotheticalMovingArc) {
  auto checker=stoppedChecker();
  for (double sign:{-1.,1.}) {
    std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,
      nav2_costmap_2d::FREE_SPACE);
    straightPath({0.,sign*.15});
    unsigned int x,y;ASSERT_TRUE(map->getCostmap()->worldToMap(sign*.1,0.,x,y));
    map->getCostmap()->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_THROW(commandWith(baseline,checker.get()),nav2_core::PlannerException);
    EXPECT_NO_THROW({
      auto fixed=commandWith(bounded,checker.get());
      EXPECT_DOUBLE_EQ(fixed.twist.linear.x,0.);
      EXPECT_DOUBLE_EQ(fixed.twist.angular.z,0.);
    });
  }
}

TEST_F(ActualPlugin, PrunedSidewaysEndpointCannotCreateAnUnplannedReversal) {
  auto checker=stoppedChecker();
  // pilot3: only the endpoint remains, lateral error ~0.205 m just outside
  // the 0.2 m cusp disk. Carrot x changes sign without a planned cusp.
  for (double sign:{-1.,1.}) {
    auto path=straightPath({-3.*sign,.01*sign});
    for (auto & p:path.poses) p.pose.position.y=-.205;
    bounded->setPlan(path);baseline->setPlan(path);
    EXPECT_GT(sign*commandAt(bounded,checker.get(),0.).twist.linear.x,0.);
    // Upstream RPP changes gear; the bounded single-direction contract must
    // instead abort so the route owner can plan from the actual pose.
    EXPECT_LT(sign*commandAt(baseline,checker.get(),.02*sign).twist.linear.x,0.);
    EXPECT_THROW(commandAt(bounded,checker.get(),.02*sign),nav2_core::PlannerException);
  }
}

TEST_F(ActualPlugin, ReachedCuspBrakesEvenIfEndpointIsBehindPlannedDirection) {
  auto checker=stoppedChecker();
  for (double sign:{-1.,1.}) {
    straightPath({-3.*sign,-.1*sign});
    auto fixed=commandAt(bounded,checker.get(),0.);
    EXPECT_DOUBLE_EQ(fixed.twist.linear.x,0.);
    EXPECT_DOUBLE_EQ(fixed.twist.angular.z,0.);
  }
}

TEST_F(ActualPlugin, ANewPlanCanExplicitlyChangeDirection) {
  auto checker=stoppedChecker();
  for (double sign:{1.,-1.,1.}) {
    straightPath({0.,sign*4.});
    EXPECT_GT(sign*commandAt(bounded,checker.get(),0.).twist.linear.x,0.);
  }
}

TEST_F(ActualPlugin, MixedDirectionPlanIsRejectedWithoutReplacingTheOldPlan) {
  straightPath({0.,4.});
  auto malformed=straightPath({0.,4.});
  auto reverse=malformed.poses.back();reverse.pose.position.x=3.;
  malformed.poses.push_back(reverse);
  EXPECT_THROW(bounded->setPlan(malformed),nav2_core::PlannerException);
  EXPECT_GT(commandAt(bounded,&goal,0.).twist.linear.x,0.);
}

TEST_F(ActualPlugin, RecordedPilot3MotionSectionsRemainAcceptable) {
  const std::string source=__FILE__;
  std::ifstream input(source.substr(0,source.find_last_of("/\\"))+
    "/fixtures/pilot3_motion_sections.csv");
  ASSERT_TRUE(input.is_open());
  std::map<int,nav_msgs::msg::Path> paths;
  std::string line;
  while (std::getline(input,line)) {
    std::istringstream row(line);int section;double x,y,heading;char comma;
    ASSERT_TRUE(static_cast<bool>(row>>section>>comma>>x>>comma>>y>>comma>>heading));
    auto & path=paths[section];path.header.frame_id="world";path.header.stamp=node->now();
    geometry_msgs::msg::PoseStamped p;p.header=path.header;
    p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation.z=std::sin(heading/2.);p.pose.orientation.w=std::cos(heading/2.);
    path.poses.push_back(p);
  }
  ASSERT_FALSE(paths.empty());
  for (const auto & entry:paths) {
    SCOPED_TRACE(entry.first);
    EXPECT_NO_THROW(bounded->setPlan(entry.second));
  }
}
