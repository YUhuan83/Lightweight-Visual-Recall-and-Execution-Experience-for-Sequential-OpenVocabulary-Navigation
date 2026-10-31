// Diagnostic only: inverse-quantized public maps, same real planner plugin.
#include "/home/ly/execution_continuity_review_20261022/v22_odometry_ingress/candidate/ackermann_smac/test/test_planner.cpp"
TEST_F(ActualPlanner, FixedHeadingCounterfactual) {
  std::ifstream input(std::getenv("HEADING_FIXTURE"));ASSERT_TRUE(input.good());
  unsigned w,h,count;double res,ox,oy;Pose start;
  input>>w>>h>>res>>ox>>oy>>start.x>>start.y>>start.yaw>>count;
  std::vector<Pose> goals(count);for(auto & p:goals)input>>p.x>>p.y>>p.yaw;
  grid->resizeMap(w,h,res,ox,oy);std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
  unsigned index;int value;while(input>>index>>value) {
    ASSERT_LT(index,w*h);grid->setCost(index%w,index/w,value<0?255:value==100?254:value==99?253:value==0?0:1+(value-1)*251/97);
  }
  configure();
  const auto original=std::vector<unsigned char>(grid->getCharMap(),grid->getCharMap()+w*h);
  for(bool free_map:{false,true}) {
    if(free_map)std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
    for(unsigned i=0;i<count;++i) {
      const auto began=Clock::now();const auto & goal=goals[i];
      try {
        auto path=planner->createPlan(stamped(start),stamped(goal));std::vector<Pose> poses;unsigned blocked=0,changes=0;int previous=0;
        for(const auto & p:path.poses) {poses.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});blocked+=!safe(*grid,map->getRobotFootprint(),poses.back(),true);}
        for(size_t j=1;j<poses.size();++j) {
          const auto a=poses[j-1],b=poses[j];const auto dx=b.x-a.x,dy=b.y-a.y;
          if(std::hypot(dx,dy)<1e-8)continue;int direction=(dx*std::cos(a.yaw)+dy*std::sin(a.yaw))>0?1:-1;
          if(previous && previous!=direction)++changes;previous=direction;
        }
        std::cout<<"HEADING_RESULT free="<<free_map<<" variant="<<i<<" ok=1 changes="<<changes<<" cost="<<maneuverCost(poses)<<" blocked="<<blocked<<" geometry="<<curvatureFeasible(poses,res,8.,72)<<" goal_error="<<std::hypot(poses.back().x-goal.x,poses.back().y-goal.y)<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;
        std::ofstream dump(std::string(std::getenv("HEADING_FIXTURE"))+".free_"+std::to_string(free_map)+".variant_"+std::to_string(i)+".path");dump<<std::setprecision(17);for(const auto & p:poses)dump<<p.x<<" "<<p.y<<" "<<p.yaw<<"\n";
      }catch(const std::exception & e){std::cout<<"HEADING_RESULT free="<<free_map<<" variant="<<i<<" ok=0 reason="<<e.what()<<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;}
    }
  }
}
