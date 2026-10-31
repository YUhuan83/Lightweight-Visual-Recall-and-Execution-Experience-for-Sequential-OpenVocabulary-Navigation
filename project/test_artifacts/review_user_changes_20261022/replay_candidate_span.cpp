// Diagnostic: actual plugin on inverse-quantized published costmap, not the private search map.
#include "/home/ly/execution_continuity_review_20261022/v4/candidate/ackermann_smac/test/test_planner.cpp"
#include "ompl/base/spaces/DubinsStateSpace.h"
TEST_F(ActualPlanner, RecordedCandidateSpan) {
  std::ifstream input(std::getenv("CANDIDATE_FIXTURE"));
  ASSERT_TRUE(input.good());
  unsigned w,h,count; double res,ox,oy; Pose start;
  input>>w>>h>>res>>ox>>oy>>start.x>>start.y>>start.yaw>>count;
  std::vector<Pose> goals(count);
  for(auto & goal:goals) input>>goal.x>>goal.y>>goal.yaw;
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
  unsigned i=0;
  for(const auto & goal:goals) {
    for(const double minimum:{.4,.001}) {
      const auto direct=connect(start,goal,12.,minimum,.05,50.);
      unsigned blocked=0;
      for(const auto & p:direct.poses)blocked+=!safe(*grid,map->getRobotFootprint(),p,true);
      std::cout<<"WORD "<<i<<" min="<<minimum<<" poses="<<direct.poses.size()
        <<" length="<<direct.length<<" cusps="<<direct.cusps<<" blocked="<<blocked<<std::endl;
    }
    ompl::base::DubinsStateSpace forward(12.,false);
    auto a=forward.allocState(),b=forward.allocState(),sample=forward.allocState();
    a->as<ompl::base::SE2StateSpace::StateType>()->setXY(start.x,start.y);
    a->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start.yaw);
    b->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal.x,goal.y);
    b->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal.yaw);
    const double length=forward.distance(a,b);unsigned blocked=0;
    std::vector<Pose> forward_poses;
    if(length<=50.) {
      const unsigned steps=static_cast<unsigned>(std::ceil(length/.05));
      for(unsigned j=0;j<=steps;++j) {
        forward.interpolate(a,b,static_cast<double>(j)/steps,sample);
        const auto p=sample->as<ompl::base::SE2StateSpace::StateType>();
        forward_poses.push_back({p->getX(),p->getY(),p->getYaw()});
        blocked+=!safe(*grid,map->getRobotFootprint(),forward_poses.back(),true);
      }
    }
    std::cout<<"FORWARD "<<i<<" length="<<length<<" poses="<<forward_poses.size()
      <<" blocked="<<blocked<<" geometry="<<curvatureFeasible(forward_poses,res,12.,72)<<std::endl;
    std::ofstream forward_dump(std::string(std::getenv("CANDIDATE_FIXTURE"))+".forward_"+std::to_string(i));
    forward_dump<<std::setprecision(17);
    for(const auto & p:forward_poses) forward_dump<<p.x<<" "<<p.y<<" "<<p.yaw<<"\n";
    std::vector<unsigned char> saved;
    for(unsigned x=168;x<=170;++x) { saved.push_back(grid->getCost(x,98)); grid->setCost(x,98,0); }
    unsigned counterfactual_blocked=0;
    for(const auto & p:forward_poses) counterfactual_blocked+=!safe(*grid,map->getRobotFootprint(),p,true);
    for(unsigned x=168;x<=170;++x) grid->setCost(x,98,saved[x-168]);
    std::cout<<"THREE_CELL_CONTROL "<<i<<" blocked="<<counterfactual_blocked<<std::endl;
    forward.freeState(a);forward.freeState(b);forward.freeState(sample);
    const auto began=Clock::now();
    try {
      const auto path=planner->createPlan(stamped(start),stamped(goal));
      std::vector<Pose> poses;unsigned blocked=0;
      for(const auto & p:path.poses) {
        poses.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
        blocked+=!safe(*grid,map->getRobotFootprint(),poses.back(),true);
      }
      std::ofstream dump(std::string(std::getenv("CANDIDATE_FIXTURE"))+".path_"+std::to_string(i));
      dump<<std::setprecision(17);
      for(const auto & p:poses)dump<<p.x<<" "<<p.y<<" "<<p.yaw<<"\n";
      std::cout<<"SPAN "<<i<<" ok poses="<<poses.size()<<" blocked="<<blocked
        <<" geometry="<<curvatureFeasible(poses,res,12.,72)<<" cost="<<maneuverCost(poses)
        <<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;
    } catch(const std::exception & error) {
      std::cout<<"SPAN "<<i<<" rejected="<<error.what()
        <<" elapsed="<<std::chrono::duration<double>(Clock::now()-began).count()<<std::endl;
    }
    ++i;
  }
}
