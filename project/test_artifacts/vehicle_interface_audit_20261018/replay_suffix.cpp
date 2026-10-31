// Diagnostic test only: actual installed plugins, closed public map, no vehicle.
#include "/home/ly/planning_phase_physical_20261018/candidate/ackermann_smac/test/test_planner.cpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "wildos_ackermann_smac_trace/planner.hpp"
TEST_F(ActualPlanner, FrozenVehicleInterfaceMatrix) {
  const char * input=std::getenv("REPLAY_INPUT");ASSERT_NE(input,nullptr);
  std::ifstream in(input);ASSERT_TRUE(in.good());
  unsigned w,h;double res,ox,oy;Pose start,goal;
  in>>w>>h>>res>>ox>>oy>>start.x>>start.y>>start.yaw>>goal.x>>goal.y>>goal.yaw;
  ASSERT_TRUE(in.good());ASSERT_EQ(res,.25);
  grid->resizeMap(w,h,res,ox,oy);
  std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
  unsigned i;int v;bool free=std::getenv("REPLAY_FREE")!=nullptr;
  while(in>>i>>v) {
    ASSERT_LT(i,w*h);
    if(!free && (v==100 || v<0))grid->setCost(i%w,i/w,v==100?254:255);
  }
  inflate();
  const double radius=std::getenv("REPLAY_RADIUS8")?8.:12.;
  ASSERT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",radius}).successful);
  if(std::getenv("REPLAY_SUFFIX_TRACE")) {
    const bool smooth=std::getenv("REPLAY_SUFFIX_RAW")==nullptr;
    node->declare_parameter("CarLike.smooth_path",smooth);
    planner=std::make_shared<nav2_smac_planner::SmacPlannerHybrid>();
    planner->configure(node,"CarLike",tf,map);planner->activate();
    Limits lim;lim.radius=radius;
    auto candidates=search(*grid,map->getRobotFootprint(),start,lim,
      Clock::now()+std::chrono::milliseconds(200));
    size_t index=0;
    for(const auto & c:candidates.curves) {
      nav_msgs::msg::Path suffix;std::string error;
      try {suffix=planner->createPlan(stamped(c.poses.back()),stamped(goal));}
      catch(const std::exception & e) {error=e.what();}
      double pos_error=-1,yaw_error=-1;std::vector<Pose> combined=c.poses;
      if(!suffix.poses.empty()) {
        const auto & p=suffix.poses.front().pose;
        pos_error=std::hypot(p.position.x-c.poses.back().x,p.position.y-c.poses.back().y);
        yaw_error=std::abs(std::remainder(tf2::getYaw(p.orientation)-c.poses.back().yaw,2.*std::acos(-1.)));
        for(size_t k=1;k<suffix.poses.size();++k) {
          const auto & q=suffix.poses[k].pose;
          combined.push_back({q.position.x,q.position.y,tf2::getYaw(q.orientation)});
        }
      }
      bool collision_free=!suffix.poses.empty();
      for(const auto & p:combined)collision_free&=safe(*grid,map->getRobotFootprint(),p,true);
      std::cout<<std::setprecision(17)<<"SUFFIX_RESULT index="<<index++<<" smooth="<<smooth
        <<" poses="<<suffix.poses.size()<<" position_join_error="<<pos_error<<" yaw_join_error="<<yaw_error
        <<" combined_curvature_ok="<<curvatureFeasible(combined,res,radius,72)
        <<" collision_free="<<collision_free<<" error="<<error<<std::endl;
    }
    return;
  }
  const bool native=std::getenv("REPLAY_NATIVE")!=nullptr;
  if(native) {
    planner=std::make_shared<nav2_smac_planner::SmacPlannerHybrid>();
    planner->configure(node,"CarLike",tf,map);planner->activate();
  } else if(std::getenv("REPLAY_TRACE")) {
    planner=std::make_shared<wildos_ackermann_smac_trace::Planner>();
    planner->configure(node,"CarLike",tf,map);planner->activate();
  } else configure();
  auto began=Clock::now();nav_msgs::msg::Path path;std::string error;
  try {path=planner->createPlan(stamped(start),stamped(goal));}
  catch(const std::exception & e) {error=e.what();}
  const double elapsed=std::chrono::duration<double>(Clock::now()-began).count();
  std::ofstream csv(std::getenv("REPLAY_OUTPUT"));csv<<std::setprecision(17)<<"x,y,yaw\n";
  std::vector<Pose> poses;
  for(const auto & p:path.poses) {
    Pose p2{p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)};
    poses.push_back(p2);csv<<p2.x<<","<<p2.y<<","<<p2.yaw<<"\n";
  }
  double length=0;for(size_t n=1;n<poses.size();++n)length+=std::hypot(poses[n].x-poses[n-1].x,poses[n].y-poses[n-1].y);
  std::cout<<std::setprecision(17)<<"MATRIX_RESULT native="<<native<<" free="<<free<<" radius="<<radius
    <<" poses="<<poses.size()<<" elapsed="<<elapsed<<" length="<<length
    <<" start_safe="<<safe(*grid,map->getRobotFootprint(),start,true)
    <<" goal_safe="<<safe(*grid,map->getRobotFootprint(),goal,true)
    <<" curvature_ok="<<(!poses.empty() && curvatureFeasible(poses,res,radius,72))
    <<" error="<<error<<std::endl;
  if(std::getenv("REPLAY_REQUIRE_SUCCESS")) {
    EXPECT_GT(poses.size(),2u)<<error;
    if(!poses.empty()) {
      EXPECT_NEAR(poses.front().x,start.x,1e-6);
      EXPECT_NEAR(poses.front().y,start.y,1e-6);
      EXPECT_NEAR(std::remainder(poses.front().yaw-start.yaw,2.*std::acos(-1.)),0.,1e-5);
      EXPECT_TRUE(curvatureFeasible(poses,res,radius,72));
      for(const auto & p:poses)EXPECT_TRUE(safe(*grid,map->getRobotFootprint(),p,true));
    }
  }
  // Test completion means the diagnostic executed, not that navigation succeeded.
  EXPECT_TRUE(std::isfinite(elapsed));
}
