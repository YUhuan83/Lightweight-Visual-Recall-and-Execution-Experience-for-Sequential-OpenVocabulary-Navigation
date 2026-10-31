// SPDX-License-Identifier: Apache-2.0
// Actual plugin, static TF and in-memory costmap. No simulator or vehicle.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <fstream>
#include <map>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <thread>
#include <chrono>
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
    auto settings=std::vector<rclcpp::Parameter>{
        {"FollowPath.desired_linear_vel",1.},{"FollowPath.lookahead_dist",3.},
        {"FollowPath.use_rotate_to_heading",false},{"FollowPath.allow_reversing",true},
        {"FollowPath.use_collision_detection",true},
        {"FollowPath.use_velocity_scaled_lookahead_dist",false},
        {"FollowPath.use_regulated_linear_velocity_scaling",false},
        {"FollowPath.use_cost_regulated_linear_velocity_scaling",false},
        {"FollowPath.approach_velocity_scaling_dist",.1},
        {"FollowPath.max_allowed_time_to_collision_up_to_carrot",3.2},
        {"FollowPath.minimum_turning_radius",8.}};
    const std::string test_name=::testing::UnitTest::GetInstance()->current_test_info()->name();
    if(test_name.find("Continuous")!=std::string::npos) {
      settings.emplace_back("FollowPath.continuous_path_execution",true);
    }
    if(test_name.find("TownCusp")!=std::string::npos || test_name.find("ReplannedTown")!=std::string::npos) {
      auto initial=[&](rclcpp::Parameter p) {
        auto found=std::find_if(settings.begin(),settings.end(),[&](const auto & v){return v.get_name()==p.get_name();});
        if(found==settings.end()) settings.push_back(p);else *found=p;
      };
      // Load the frozen field configuration BEFORE configure(). Successful
      // parameter setters alone do not prove that a cached plugin uses them.
      for(const auto & p:std::vector<rclcpp::Parameter>{
          {"FollowPath.desired_linear_vel",1.2},{"FollowPath.min_lookahead_dist",3.},
          {"FollowPath.max_lookahead_dist",6.},{"FollowPath.lookahead_time",2.5},
          {"FollowPath.use_velocity_scaled_lookahead_dist",true},
          {"FollowPath.use_regulated_linear_velocity_scaling",true},
          {"FollowPath.regulated_linear_scaling_min_radius",8.},
          {"FollowPath.regulated_linear_scaling_min_speed",.3},
          {"FollowPath.approach_velocity_scaling_dist",2.},
          {"FollowPath.min_approach_linear_velocity",.2},
          {"FollowPath.max_allowed_time_to_collision_up_to_carrot",1.}}) initial(p);
      if(test_name.find("ReplannedTown")!=std::string::npos)
        initial({"FollowPath.use_velocity_scaled_lookahead_dist",false});
      if(const auto setting=std::getenv("WILDOS_AUDIT_LOOKAHEAD")) {
        const double look=std::stod(setting);ASSERT_GT(look,0.);ASSERT_TRUE(std::isfinite(look));
        initial({"FollowPath.lookahead_dist",look});
        initial({"FollowPath.use_velocity_scaled_lookahead_dist",false});
      }
    }
    for (const auto & p:settings) {
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

TEST_F(ActualPlugin, TrackingGeometryIsFrozenAcrossDynamicParameterUpdates) {
  for(const auto & p:std::vector<rclcpp::Parameter>{
    {"FollowPath.lookahead_dist",6.},
    {"FollowPath.use_velocity_scaled_lookahead_dist",true},
    {"FollowPath.max_robot_pose_search_dist",20.}}) {
    auto rejected=node->set_parameters({p});
    ASSERT_EQ(rejected.size(),1u);EXPECT_FALSE(rejected[0].successful);
  }
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.lookahead_dist").as_double(),3.);
  EXPECT_FALSE(node->get_parameter("FollowPath.use_velocity_scaled_lookahead_dist").as_bool());
}

TEST_F(ActualPlugin, PhysicalCommandPublishesGenerationTimeRobotFrameAndExactTwist) {
  std::vector<geometry_msgs::msg::TwistStamped> received;
  auto subscription=node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/wildos/cmd_vel_stamped",rclcpp::QoS(1),
    [&](const geometry_msgs::msg::TwistStamped::SharedPtr message){received.push_back(*message);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  for(int i=0;i<20;++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  setPath(1.);
  const auto before=node->now();const auto result=command(bounded);const auto after=node->now();
  for(int i=0;i<100 && received.empty();++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  ASSERT_EQ(received.size(),1u);
  EXPECT_EQ(received.back().header.frame_id,"base");
  EXPECT_GE(rclcpp::Time(received.back().header.stamp).nanoseconds(),before.nanoseconds());
  EXPECT_LE(rclcpp::Time(received.back().header.stamp).nanoseconds(),after.nanoseconds());
  EXPECT_DOUBLE_EQ(received.back().twist.linear.x,result.twist.linear.x);
  EXPECT_DOUBLE_EQ(received.back().twist.angular.z,result.twist.angular.z);
  auto stamp=rclcpp::Time(received.back().header.stamp).nanoseconds();
  setPath(-1.);auto reverse=command(bounded);
  for(int i=0;i<100 && received.size()<2;++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  ASSERT_EQ(received.size(),2u);
  EXPECT_GT(rclcpp::Time(received.back().header.stamp).nanoseconds(),stamp);
  EXPECT_DOUBLE_EQ(received.back().twist.linear.x,reverse.twist.linear.x);
  executor.remove_node(node->get_node_base_interface());
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

TEST_F(ActualPlugin, RecordedTownCuspRemainsKnownUntrackableCounterexample) {
  // Real submitted path and actual Bridge dispatch pose, rather than a path
  // acceptance-only check. The plant is a radius-bounded ideal bicycle; this
  // isolates geometry/feedback delay and does not establish BeamNG success.
  const std::string source=__FILE__;
  std::ifstream input(source.substr(0,source.find_last_of("/\\"))+
    "/fixtures/town_first_cusp_section_20261014.csv");
  ASSERT_TRUE(input.is_open());
  nav_msgs::msg::Path path;path.header.frame_id="world";path.header.stamp=node->now();
  struct State {double x,y,yaw,v=0.,w=0.;};
  State start{};std::string line;std::getline(input,line);
  while(std::getline(input,line)) {
    std::istringstream row(line);std::string kind;double x,y,h;char comma;
    std::getline(row,kind,',');ASSERT_TRUE(static_cast<bool>(row>>x>>comma>>y>>comma>>h));
    if(kind=="start") {start={x,y,h};continue;}
    geometry_msgs::msg::PoseStamped p;p.header=path.header;
    p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation.z=std::sin(h/2.);p.pose.orientation.w=std::cos(h/2.);
    path.poses.push_back(p);
  }
  ASSERT_EQ(path.poses.size(),50u);
  if(const auto setting=std::getenv("WILDOS_AUDIT_PATH_SCALE")) {
    const double scale=std::stod(setting);ASSERT_GT(scale,0.);ASSERT_TRUE(std::isfinite(scale));
    const auto anchor=path.poses.front().pose.position;
    for(auto & p:path.poses) {
      p.pose.position.x=anchor.x+scale*(p.pose.position.x-anchor.x);
      p.pose.position.y=anchor.y+scale*(p.pose.position.y-anchor.y);
    }
  }
  if(std::getenv("WILDOS_AUDIT_NOMINAL_START")) {
    const auto & p=path.poses.front().pose;
    start={p.position.x,p.position.y,std::atan2(2.*p.orientation.w*p.orientation.z,
      1.-2.*p.orientation.z*p.orientation.z)};
  }
  map->getCostmap()->resizeMap(2400,2400,.025,700.,-40.);
  std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+5760000,
    nav2_costmap_2d::FREE_SPACE);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(const auto & xy:std::vector<std::pair<double,double>>{{3.4,1.05},{3.4,-1.05},{-1.05,-1.05},{-1.05,1.05}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  map->setRobotFootprint(footprint);
  auto checker=stoppedChecker();const auto end=path.poses.back().pose;
  auto asPose=[&](const State & s) {
    geometry_msgs::msg::PoseStamped p;p.header=path.header;p.header.stamp=node->now();
    p.pose.position.x=s.x;p.pose.position.y=s.y;
    p.pose.orientation.z=std::sin(s.yaw/2.);p.pose.orientation.w=std::cos(s.yaw/2.);return p;
  };
  constexpr double dt=.05;
  for(double delay:{0.,.1,.2,.4}) {
    SCOPED_TRACE(delay);bounded->setPlan(path);checker->reset();
    std::vector<State> history{start};bool reached=false;std::string error;
    double closest=100.,traveled=0.;
    for(int tick=0;tick<2400;++tick) {
      const auto actual=history.back();auto actual_pose=asPose(actual);
      geometry_msgs::msg::Twist actual_speed;actual_speed.linear.x=actual.v;actual_speed.angular.z=actual.w;
      closest=std::min(closest,std::hypot(actual.x-end.position.x,actual.y-end.position.y));
      if(checker->isGoalReached(actual_pose.pose,end,actual_speed)) {reached=true;break;}
      const size_t lag=static_cast<size_t>(std::lround(delay/dt));
      const auto sampled=history[history.size()>lag?history.size()-1-lag:0];
      geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="world";
      transform.child_frame_id="base";transform.transform.translation.x=sampled.x;
      transform.transform.translation.y=sampled.y;transform.transform.rotation=asPose(sampled).pose.orientation;
      tf->setTransform(transform,"tracking_replay",true);
      geometry_msgs::msg::Twist measured;measured.linear.x=sampled.v;measured.angular.z=sampled.w;
      geometry_msgs::msg::TwistStamped command;
      auto sampled_pose=asPose(sampled);
      sampled_pose.header.stamp=node->now()-rclcpp::Duration::from_seconds(delay);
      try {command=bounded->computeVelocityCommands(sampled_pose,measured,checker.get());}
      catch(const std::exception & exception) {error=exception.what();break;}
      const double v=command.twist.linear.x,w=command.twist.angular.z;
      EXPECT_LE(std::abs(w),std::abs(v)/8.+1e-9);
      const double turn=w*dt,k=std::abs(v)>1e-10?w/v:0.;
      const double dx=std::abs(k)>1e-10?std::sin(turn)/k:v*dt;
      const double dy=std::abs(k)>1e-10?(1.-std::cos(turn))/k:0.;
      history.push_back({actual.x+dx*std::cos(actual.yaw)-dy*std::sin(actual.yaw),
        actual.y+dx*std::sin(actual.yaw)+dy*std::cos(actual.yaw),actual.yaw+turn,v,w});
      traveled+=std::abs(v)*dt;
    }
    const auto actual=history.back();
    std::cout<<"TRACKING_REPLAY: {\"delay_sec\":"<<delay<<",\"reached\":"<<(reached?"true":"false")
      <<",\"closest_m\":"<<closest<<",\"final_error_m\":"
      <<std::hypot(actual.x-end.position.x,actual.y-end.position.y)<<",\"traveled_m\":"<<traveled
      <<",\"heading_error_rad\":"<<std::abs(std::remainder(actual.yaw-
        std::atan2(2.*end.orientation.w*end.orientation.z,1.-2.*end.orientation.z*end.orientation.z),2.*std::acos(-1.)))
      <<",\"final_speed_m_s\":"<<actual.v
      <<",\"error\":"<<std::quoted(error)<<"}"<<std::endl;
    if(std::getenv("WILDOS_AUDIT_PATH_SCALE")) EXPECT_TRUE(reached);
    else {
      // Admission must reject this preserved field counterexample. Merely
      // removing the direction guard would introduce an unplanned gear change.
      EXPECT_FALSE(reached);
      EXPECT_GT(closest,.2);
      EXPECT_NE(error.find("reverse a single-direction path"),std::string::npos);
    }
  }
}

TEST_F(ActualPlugin, ReplannedTownSameGoalReachesEachStoppedCusp) {
  const std::string source=__FILE__;
  std::ifstream input(source.substr(0,source.find_last_of("/\\"))+
    "/fixtures/town_same_goal_replanned12_20261014.csv");
  ASSERT_TRUE(input.is_open());
  struct State {double x,y,yaw,v=0.,w=0.;};
  State start{};std::string line;std::getline(input,line);
  nav_msgs::msg::Path path;path.header.frame_id="world";path.header.stamp=node->now();
  while(std::getline(input,line)) {
    std::istringstream row(line);std::string kind;double x,y,h;char comma;
    std::getline(row,kind,',');ASSERT_TRUE(static_cast<bool>(row>>x>>comma>>y>>comma>>h));
    if(kind=="start") {start={x,y,h};continue;}
    geometry_msgs::msg::PoseStamped p;p.header=path.header;
    p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation.z=std::sin(h/2.);p.pose.orientation.w=std::cos(h/2.);
    path.poses.push_back(p);
  }
  ASSERT_GT(path.poses.size(),2u);
  EXPECT_NEAR(path.poses.back().pose.position.x,712.0949878692627,.2);
  EXPECT_NEAR(path.poses.back().pose.position.y,-3.3905601501464844,.2);
  std::vector<size_t> boundary{0};
  for(size_t i=1;i+1<path.poses.size();++i) {
    const auto & a=path.poses[i-1].pose.position;
    const auto & b=path.poses[i].pose.position;
    const auto & c=path.poses[i+1].pose.position;
    if((b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y)<0.) boundary.push_back(i);
  }
  boundary.push_back(path.poses.size()-1);
  ASSERT_EQ(boundary.size(),5u);
  map->getCostmap()->resizeMap(3200,3200,.025,690.,-40.);
  std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+10240000,
    nav2_costmap_2d::FREE_SPACE);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(const auto & xy:std::vector<std::pair<double,double>>{{3.4,1.05},{3.4,-1.05},{-1.05,-1.05},{-1.05,1.05}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  map->setRobotFootprint(footprint);auto checker=stoppedChecker();
  auto asPose=[&](const State & s) {
    geometry_msgs::msg::PoseStamped p;p.header=path.header;p.header.stamp=node->now();
    p.pose.position.x=s.x;p.pose.position.y=s.y;
    p.pose.orientation.z=std::sin(s.yaw/2.);p.pose.orientation.w=std::cos(s.yaw/2.);return p;
  };
  constexpr double dt=.05;
  for(double delay:{0.,.1,.2,.4}) {
    State actual=start;
    for(size_t section=0;section+1<boundary.size();++section) {
      SCOPED_TRACE(delay);
      SCOPED_TRACE(section);
      auto local=path;
      local.poses.assign(path.poses.begin()+boundary[section],path.poses.begin()+boundary[section+1]+1);
      bounded->setPlan(local);checker->reset();const auto end=local.poses.back().pose;
      std::vector<State> history{actual};bool reached=false;std::string error;
      for(int tick=0;tick<2400;++tick) {
        actual=history.back();geometry_msgs::msg::Twist speed;
        speed.linear.x=actual.v;speed.angular.z=actual.w;
        if(checker->isGoalReached(asPose(actual).pose,end,speed)) {reached=true;break;}
        const size_t lag=static_cast<size_t>(std::lround(delay/dt));
        const auto sampled=history[history.size()>lag?history.size()-1-lag:0];
        geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="world";
        transform.child_frame_id="base";transform.transform.translation.x=sampled.x;
        transform.transform.translation.y=sampled.y;transform.transform.rotation=asPose(sampled).pose.orientation;
        tf->setTransform(transform,"replanned_tracking",true);
        auto sampled_pose=asPose(sampled);
        sampled_pose.header.stamp=node->now()-rclcpp::Duration::from_seconds(delay);
        geometry_msgs::msg::Twist measured;measured.linear.x=sampled.v;measured.angular.z=sampled.w;
        geometry_msgs::msg::TwistStamped command;
        try {command=bounded->computeVelocityCommands(sampled_pose,measured,checker.get());}
        catch(const std::exception & exception) {error=exception.what();break;}
        const double v=command.twist.linear.x,w=command.twist.angular.z;
        EXPECT_LE(std::abs(w),std::abs(v)/8.+1e-9);
        const double turn=w*dt,k=std::abs(v)>1e-10?w/v:0.;
        const double dx=std::abs(k)>1e-10?std::sin(turn)/k:v*dt;
        const double dy=std::abs(k)>1e-10?(1.-std::cos(turn))/k:0.;
        history.push_back({actual.x+dx*std::cos(actual.yaw)-dy*std::sin(actual.yaw),
          actual.y+dx*std::sin(actual.yaw)+dy*std::cos(actual.yaw),actual.yaw+turn,v,w});
      }
      std::cout<<"REPLANNED_TRACKING: {\"delay_sec\":"<<delay<<",\"section\":"<<section
        <<",\"reached\":"<<(reached?"true":"false")<<",\"final_error_m\":"
        <<std::hypot(actual.x-end.position.x,actual.y-end.position.y)
        <<",\"final_speed_m_s\":"<<actual.v<<",\"error\":"<<std::quoted(error)<<"}"<<std::endl;
      ASSERT_TRUE(reached);
      EXPECT_LE(std::abs(actual.v),.05);
    }
  }
}
#include "continuous_cases.inc"
