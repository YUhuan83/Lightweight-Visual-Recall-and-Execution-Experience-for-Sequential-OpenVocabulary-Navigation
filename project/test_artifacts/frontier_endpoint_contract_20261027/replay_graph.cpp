#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <rclcpp/serialization.hpp>
#include "graphnav_planner/planner.hpp"
#include "graphnav_planner/graph_score_overlay.hpp"
using Graph=graphnav_msgs::msg::NavigationGraph;
Graph::SharedPtr load(const std::string& path) {
  std::ifstream f(path,std::ios::binary); if(!f.good())throw std::runtime_error(path);
  std::vector<char> bytes((std::istreambuf_iterator<char>(f)),{});
  rclcpp::SerializedMessage data(bytes.size());auto& raw=data.get_rcl_serialized_message();
  std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
  auto g=std::make_shared<Graph>();rclcpp::Serialization<Graph>().deserialize_message(&data,g.get());return g;
}
int main(int argc,char** argv) {
  if(argc!=2 && argc!=3)return 2;
  std::ifstream list(argv[1]);std::string raw,scored;double x,y,z;int64_t stamp;
  std::cout<<std::setprecision(17);
  while(list>>raw>>scored>>x>>y>>z>>stamp) {
    graphnav_planner::GraphScoreOverlay overlay;overlay.remember(load(raw));
    auto graph=overlay.combine(*load(scored));if(!graph)throw std::runtime_error("unpaired topology");
    graphnav_planner::Planner p(rclcpp::get_logger("replay"));p.set_trav_class("default");
    p.frontier_dist_cost_factor_=2.;p.goal_dist_cost_factor_=1.;p.frontier_score_factor_=20.;
    p.min_local_frontier_score_=.4;p.local_frontier_radius_=10.;p.path_smoothness_period_=0.;
    p.physical_risk_factor_=10.;p.dynamic_risk_factor_=12.;p.stale_observation_seconds_=5.;
    p.low_confidence_factor_=4.;p.heading_turn_cost_factor_=2.;p.robot_profile_id_="default";
    if(argc==3) {p.frontier_score_factor_=0.;p.frontier_dist_cost_factor_=1.;}
    p.update_graph(graph);Eigen::Vector3d goal(x,y,z);
    auto path=p.plan_to_goal(goal,6.,rclcpp::Time(stamp,RCL_ROS_TIME));
    if(scored.find("pair_000156_")!=std::string::npos) {
      for(const auto& m:p.get_score_visualization(rclcpp::Time(stamp,RCL_ROS_TIME),"spot1/odom",true).markers)
        if(m.ns=="frontier_id_text" && !m.text.empty())
          std::cout<<"COST "<<m.pose.position.x<<" "<<m.pose.position.y<<" "<<m.text<<std::endl;
    }
    std::cout<<"RESULT "<<scored<<" "<<stamp<<" "<<p.route_complete()<<" "<<path.size();
    if(!path.empty()) {
      const auto a=path.front(),b=path.back();
      std::cout<<" "<<a.x()<<" "<<a.y()<<" "<<b.x()<<" "<<b.y()<<" "<<(a-goal).head<2>().norm()<<" "<<(b-goal).head<2>().norm();
    }
    std::cout<<std::endl;
  }
}
