// Diagnostic only: recorded raw costmaps, real 12m planner, no action/vehicle.
#include "/home/ly/execution_continuity_review_20261022/v33_integrated_progress/candidate/ackermann_smac/test/test_planner.cpp"
TEST_F(ActualPlanner, RecordedObservationPoses) {
  std::ifstream input(std::getenv("OBSERVATION_FIXTURE"));ASSERT_TRUE(input.good());
  unsigned w,h,count;double res,ox,oy;Pose start;
  input>>w>>h>>res>>ox>>oy>>start.x>>start.y>>start.yaw>>count;
  std::vector<Pose> goals(count);for(auto & p:goals)input>>p.x>>p.y>>p.yaw;
  grid->resizeMap(w,h,res,ox,oy);std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
  unsigned index;int value;while(input>>index>>value){ASSERT_LT(index,w*h);grid->setCost(index%w,index/w,value);}
  ASSERT_TRUE(node->set_parameter({"CarLike.minimum_turning_radius",12.}).successful);
  configure();
  for(unsigned i=0;i<count;++i) {
    const auto began=Clock::now();
    try {
      auto path=planner->createPlan(stamped(start),stamped(goals[i]));std::vector<Pose> poses;unsigned blocked=0,cusps=0;int previous=0;
      for(const auto & p:path.poses){poses.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});blocked+=!safe(*grid,map->getRobotFootprint(),poses.back(),true);}
      for(size_t j=1;j<poses.size();++j){const auto a=poses[j-1],b=poses[j];const double dx=b.x-a.x,dy=b.y-a.y;if(std::hypot(dx,dy)<1e-8)continue;int sign=dx*std::cos(a.yaw)+dy*std::sin(a.yaw)>0?1:-1;if(previous && sign!=previous)++cusps;previous=sign;}
      std::cout<<"OBS_RESULT variant="<<i<<" ok=1 cusps="<<cusps<<" cost="<<maneuverCost(poses)<<" blocked="<<blocked<<" geometry="<<curvatureFeasible(poses,res,12.,72)<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;
    }catch(const std::exception & e){std::cout<<"OBS_RESULT variant="<<i<<" ok=0 reason="<<e.what()<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;}
  }
}
