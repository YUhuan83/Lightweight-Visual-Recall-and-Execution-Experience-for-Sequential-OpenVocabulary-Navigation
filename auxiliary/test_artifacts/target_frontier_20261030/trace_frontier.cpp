// Diagnostic only: expose existing planner decisions, never installed.
#include <bits/stdc++.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <Eigen/Dense>
#include <graaflib/graph.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <graphnav_msgs/msg/navigation_graph.hpp>
#include <graphnav_msgs/msg/uuid.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#define private public
#include "graphnav_planner/planner.hpp"
#undef private
#include <graaflib/algorithm/shortest_path/dijkstra_shortest_path.h>
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  std::ifstream file(argv[1],std::ios::binary);
  std::vector<char> data((std::istreambuf_iterator<char>(file)),{});
  rclcpp::SerializedMessage serialized(data.size());
  auto &buf=serialized.get_rcl_serialized_message();
  std::memcpy(buf.buffer,data.data(),data.size());buf.buffer_length=data.size();
  auto g=std::make_shared<graphnav_msgs::msg::NavigationGraph>();
  rclcpp::Serialization<graphnav_msgs::msg::NavigationGraph>().deserialize_message(&serialized,g.get());
  Eigen::Vector3d goal(std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4]));
  const int64_t stamp=std::stoll(argv[5]);
  const std::string mode=argc>6?argv[6]:"baseline";
  graphnav_planner::Planner p(rclcpp::get_logger("frontier_trace"));p.set_trav_class("default");
  p.path_smoothness_period_=0.;p.frontier_score_factor_=20.;
  if(mode=="edge_time_now")g->header.stamp=rclcpp::Time(stamp).to_msg();
  if(mode=="no_visual_cost")p.frontier_score_factor_=0.;
  if(mode=="no_physical_cost")p.physical_risk_factor_=p.dynamic_risk_factor_=p.low_confidence_factor_=0.;
  if(mode=="no_heading")for(auto&n:g->nodes)n.heading_candidates.clear();
  if(mode=="no_physical_filters"){
    for(auto&n:g->nodes){n.robot_traversability.clear();n.heading_candidates.clear();}
    for(auto&e:g->edges)e.robot_traversability.clear();
  }
  if(mode=="reverse_topology"){
    auto edges=g->edges;
    for(auto e:edges){std::swap(e.from_idx,e.to_idx);g->edges.push_back(e);}
  }
  p.update_graph(g);auto path=p.plan_to_goal(goal,6.,rclcpp::Time(stamp));
  std::cout<<std::setprecision(15)<<"MODE "<<mode<<"\nPATH";
  for(const auto&v:path)std::cout<<" "<<v.x()<<","<<v.y()<<","<<v.z();
  std::cout<<"\n";
  std::cout<<"CURRENT "<<p.current_node_idx_<<"\n";
  for(size_t i=0;i<g->nodes.size();++i){
    const auto&n=g->nodes[i];
    if(p.trav_class_idx_>=n.trav_properties.size() || !n.trav_properties[p.trav_class_idx_].is_frontier)continue;
    const auto scored=p.frontier_scores_.find(i);
    bool reachable=bool(graaf::algorithm::dijkstra_shortest_path(p.graph_,p.current_node_idx_,i));
    std::cout<<"CAND "<<i<<" "<<n.pose.position.x<<" "<<n.pose.position.y<<" "<<n.pose.position.z<<" "<<reachable;
    if(scored!=p.frontier_scores_.end())std::cout<<" "<<scored->second.second.first<<" "<<scored->second.second.second;
    else std::cout<<" -1 -1";
    std::cout<<"\n";
  }
  for(const auto&e:g->edges){
    const auto&n=g->nodes[e.to_idx];
    if(n.pose.position.x<730||n.pose.position.x>745||n.pose.position.y> -75||n.pose.position.y< -90)continue;
    bool reachable=bool(graaf::algorithm::dijkstra_shortest_path(p.graph_,p.current_node_idx_,e.from_idx));
    double cost=p.trav_class_idx_<e.traversability.size()?e.traversability[p.trav_class_idx_].traversability_cost:-1.;
    std::cout<<"EDGE "<<e.from_idx<<" "<<e.to_idx<<" "<<reachable<<" "<<cost;
    for(const auto&a:e.robot_traversability)
      std::cout<<" a="<<a.physical_traversability<<","<<a.dynamic_risk<<","<<a.confidence;
    std::cout<<"\n";
  }
  rclcpp::shutdown();return 0;
}
