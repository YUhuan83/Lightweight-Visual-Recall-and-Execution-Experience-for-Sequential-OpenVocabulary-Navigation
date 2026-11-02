// Replay a recorded raw costmap through the public Nav2 plugin API.
#include <fstream>
#include <iostream>
#include <algorithm>
#include "wildos_ackermann_smac/planner.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

int main(int argc,char ** argv) {
  if(argc!=2)return 2;
  std::ifstream input(argv[1]);
  unsigned w,h,count;double resolution,ox,oy;
  wildos_ackermann_smac::Pose start;
  input>>w>>h>>resolution>>ox>>oy>>start.x>>start.y>>start.yaw>>count;
  if(!input || !w || !h || count>20)return 2;
  std::vector<wildos_ackermann_smac::Pose> goals(count);
  for(auto & p:goals)input>>p.x>>p.y>>p.yaw;
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("cleanroom_replay_planner");
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("cleanroom_replay_map");
  map->set_parameters({{"plugins",std::vector<std::string>{"inflation_layer"}},
    {"global_frame","spot1/odom"},{"robot_base_frame","spot1/nav_base_link"},
    {"resolution",resolution},{"width",80},{"height",80},
    {"track_unknown_space",false},{"footprint_padding",0.},
    {"footprint","[[3.41,1.06],[3.41,-1.06],[-1.06,-1.06],[-1.06,1.06]]"}});
  map->declare_parameter("inflation_layer.plugin","nav2_costmap_2d::InflationLayer");
  map->declare_parameter("inflation_layer.inflation_radius",3.75);
  map->declare_parameter("inflation_layer.cost_scaling_factor",2.);
  map->configure();
  auto grid=map->getCostmap();grid->resizeMap(w,h,resolution,ox,oy);
  std::fill(grid->getCharMap(),grid->getCharMap()+w*h,0);
  unsigned index;int cost;
  while(input>>index>>cost) {
    if(index>=w*h || cost<0 || cost>255)return 2;
    grid->setCost(index%w,index/w,cost);
  }
  for(const auto & p:std::vector<rclcpp::Parameter>{
    {"CarLike.motion_model_for_search","REEDS_SHEPP"},{"CarLike.minimum_turning_radius",12.},
    {"CarLike.angle_quantization_bins",72},{"CarLike.allow_unknown",true},
    {"CarLike.tolerance",1.},{"CarLike.max_iterations",300000},
    {"CarLike.max_on_approach_iterations",1000},{"CarLike.max_planning_time",1.5},
    {"CarLike.analytic_expansion_ratio",3.5},{"CarLike.analytic_expansion_max_length",50.},
    {"CarLike.non_straight_penalty",1.2},{"CarLike.cost_penalty",2.},{"CarLike.lookup_table_size",20.}})
    node->declare_parameter(p.get_name(),p.get_parameter_value());
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto planner=std::make_shared<wildos_ackermann_smac::Planner>();
  planner->configure(node,"CarLike",tf,map);planner->activate();
  const auto stamped=[](const auto & p){
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="spot1/odom";
    pose.pose.position.x=p.x;pose.pose.position.y=p.y;
    tf2::Quaternion q;q.setRPY(0,0,p.yaw);pose.pose.orientation=tf2::toMsg(q);return pose;
  };
  unsigned successes=0;
  for(unsigned i=0;i<count;++i) {
    try {
      auto path=planner->createPlan(stamped(start),stamped(goals[i]));
      unsigned blocked=0;
      for(const auto & p:path.poses)if(!wildos_ackermann_smac::safe(*grid,map->getRobotFootprint(),
        {p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)},true))++blocked;
      const bool ok=path.poses.size()>1 && blocked==0;
      successes+=ok;
      std::cout<<"REPLAY_RESULT {\"candidate\":"<<i<<",\"poses\":"<<path.poses.size()
        <<",\"blocked\":"<<blocked<<",\"passed\":"<<(ok?"true":"false")<<"}"<<std::endl;
    }catch(const std::exception & error){
      std::cout<<"REPLAY_RESULT {\"candidate\":"<<i<<",\"passed\":false}"<<std::endl;
      std::cerr<<error.what()<<std::endl;
    }
  }
  planner->deactivate();planner->cleanup();planner.reset();map->cleanup();map.reset();
  rclcpp::shutdown();return successes?0:1;
}
