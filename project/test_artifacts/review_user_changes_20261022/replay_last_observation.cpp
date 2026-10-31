// Isolated actual-plugin probe of the archived public OccupancyGrid.
// Public inflation is inverse-quantized; this is not the exact private map.
#include "/home/ly/execution_continuity_review_20261022/v4/candidate/ackermann_smac/test/test_planner.cpp"
class UnsmoothNative : public nav2_smac_planner::SmacPlannerHybrid {
public:
  nav_msgs::msg::Path rawPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) {
    auto smoother=std::move(_smoother);
    try { auto path=createPlan(start,goal); _smoother=std::move(smoother); return path; }
    catch(...) { _smoother=std::move(smoother); throw; }
  }
};
TEST_F(ActualPlanner, RecordedObservationEndpointProbe) {
  const auto fixture=std::getenv("OBSERVATION_FIXTURE"); ASSERT_NE(fixture,nullptr);
  std::ifstream input(fixture); ASSERT_TRUE(input.good());
  unsigned w,h;double res,ox,oy;Pose start,goal;
  input>>w>>h>>res>>ox>>oy>>start.x>>start.y>>start.yaw>>goal.x>>goal.y>>goal.yaw;
  ASSERT_TRUE(input.good());grid->resizeMap(w,h,res,ox,oy);
  std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
  unsigned index;int value;
  while(input>>index>>value) {
    ASSERT_LT(index,w*h);
    grid->setCost(index%w,index/w,value<0?255:value==100?254:value==99?253:
      value==0?0:1+(value-1)*251/97);
  }
  ASSERT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",12.}).successful);
  configure();
  const auto body=map->getRobotFootprint();
  std::cout<<"REPLAY start="<<start.x<<","<<start.y<<","<<start.yaw
    <<" goal="<<goal.x<<","<<goal.y<<","<<goal.yaw
    <<" start_safe="<<safe(*grid,body,start,true)
    <<" goal_safe="<<safe(*grid,body,goal,true)<<std::endl;
  const auto curve=connect(start,goal,12.,.4,.05,50.);
  unsigned blocked=0;
  for(const auto & p:curve.poses) {blocked+=!safe(*grid,body,p,true);}
  std::cout<<"REPLAY direct poses="<<curve.poses.size()<<" cost="<<maneuverCost(curve.poses)
    <<" geometry="<<curvatureFeasible(curve.poses,res,12.,72)
    <<" blocked="<<blocked<<std::endl;
  // Diagnosis only: identify whether an exact shortest word was excluded by
  // its tiny motion sections. Do not use it as an executable plan.
  const auto micro=connect(start,goal,12.,.001,.05,50.);
  std::cout<<"REPLAY micro_word poses="<<micro.poses.size()<<" cusps="<<micro.cusps
    <<" cost="<<maneuverCost(micro.poses)<<std::endl;
  try {
    const auto path=planner->createPlan(stamped(start),stamped(goal));
    std::vector<Pose> poses;
    for(const auto & p:path.poses)poses.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
    std::cout<<"REPLAY planner poses="<<poses.size()<<" cost="<<maneuverCost(poses)
      <<" remaining="<<std::hypot(poses.back().x-goal.x,poses.back().y-goal.y)<<std::endl;
  } catch(const std::exception & error) {
    std::cout<<"REPLAY rejected="<<error.what()<<std::endl;
  }
  UnsmoothNative native;native.configure(node,"CarLike",tf,map);native.activate();
  Limits limits;limits.radius=12.;limits.bins=72;
  const auto anchors=search(*grid,body,start,limits,Clock::now()+std::chrono::milliseconds(200));
  unsigned number=0;
  for(const auto & anchor:anchors.curves) {
    try {
      const auto path=native.rawPlan(stamped(anchor.poses.back()),stamped(goal));
      std::vector<Pose> poses;unsigned blocked=0;
      for(const auto & p:path.poses) {
        poses.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
        blocked+=!safe(*grid,body,poses.back(),true);
      }
      std::ofstream dump(std::string(fixture)+".anchor_"+std::to_string(number)+".txt");
      dump<<std::setprecision(17);
      for(const auto & p:poses)dump<<p.x<<" "<<p.y<<" "<<p.yaw<<"\n";
      std::cout<<"REPLAY anchor="<<number<<" poses="<<poses.size()<<" blocked="<<blocked
        <<" geometry="<<curvatureFeasible(poses,res,12.,72)<<" gap="
        <<(poses.empty()?-1.:std::hypot(poses.back().x-goal.x,poses.back().y-goal.y))<<std::endl;
    } catch(const std::exception & error) {
      std::cout<<"REPLAY anchor="<<number<<" rejected="<<error.what()<<std::endl;
    }
    ++number;
  }
  native.deactivate();native.cleanup();
}
