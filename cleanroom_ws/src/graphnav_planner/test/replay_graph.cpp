// Public planner API only: deserialize a real graph and emit its planned path.
#include <fstream>
#include <cstring>
#include <iostream>
#include <iterator>
#include <rclcpp/serialization.hpp>
#include "graphnav_planner/planner.hpp"

int main(int argc,char ** argv) {
  if(argc!=5)return 2;
  std::ifstream input(argv[1],std::ios::binary);
  std::vector<char> bytes((std::istreambuf_iterator<char>(input)),{});
  if(bytes.empty())return 2;
  rclcpp::SerializedMessage serialized(bytes.size());
  auto & raw=serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
  auto graph=std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  rclcpp::Serialization<graphnav_msgs::msg::NavigationGraph>().deserialize_message(&serialized,graph.get());
  graphnav_planner::Planner planner(rclcpp::get_logger("cleanroom_graph_replay"));
  planner.set_trav_class("default");planner.path_smoothness_period_=0.;
  planner.update_graph(graph);
  Eigen::Vector3d goal(std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4]));
  auto path=planner.plan_to_goal(goal,3.,rclcpp::Time(graph->header.stamp),graph);
  std::cout<<"REPLAY_PATH [";
  for(size_t i=0;i<path.size();++i){if(i)std::cout<<",";std::cout<<"["<<path[i].x()<<","<<path[i].y()<<","<<path[i].z()<<"]";}
  std::cout<<"]"<<std::endl;
  return path.empty()?1:0;
}
