#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <graphnav_planner/planner.hpp>
#include <fstream>
#include <cstring>
#include <iostream>

int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  if(argc!=4) return 2;
  std::ifstream input(argv[1],std::ios::binary);
  std::vector<char> data((std::istreambuf_iterator<char>(input)),{});
  if(data.empty()) return 3;
  rclcpp::SerializedMessage serialized(data.size());
  auto& bytes=serialized.get_rcl_serialized_message();
  std::memcpy(bytes.buffer,data.data(),data.size());bytes.buffer_length=data.size();
  auto graph=std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  rclcpp::Serialization<graphnav_msgs::msg::NavigationGraph>().deserialize_message(&serialized,graph.get());
  const double ox=std::stod(argv[2]),oy=std::stod(argv[3]);
  const auto original=graph->current_node_idx;
  std::cout<<"start,target,original_start,start_gap_m,start_feasible,route_points,complete\n";
  for(size_t start=0;start<graph->nodes.size();++start) {
    const auto& n=graph->nodes[start];
    const double gap=std::hypot(n.pose.position.x-ox,n.pose.position.y-oy);
    if(start!=original && gap>4.) continue;
    bool feasible=true;
    for(const auto& a:n.robot_traversability) if(a.robot_profile_id=="default")feasible=a.feasible;
    graph->current_node_idx=start;
    for(size_t target=0;target<graph->nodes.size();++target) {
      const auto& candidate=graph->nodes[target];
      if(candidate.trav_properties.empty() || !candidate.trav_properties[0].is_frontier) continue;
      graphnav_planner::Planner planner(rclcpp::get_logger("captured_graph_replay"));
      planner.set_trav_class("default");planner.update_graph(graph);
      auto p=candidate.pose.position;Eigen::Vector3d goal(p.x,p.y,p.z);
      auto path=planner.plan_to_goal(goal,6.,rclcpp::Time(graph->header.stamp));
      std::cout<<start<<","<<target<<","<<(start==original)<<","<<gap<<","<<feasible<<","<<path.size()<<","<<planner.route_complete()<<"\n";
    }
  }
  rclcpp::shutdown();return 0;
}
