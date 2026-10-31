// Component replay of saved terrain/TF/graph/odom; no simulator or DDS traffic.
#include <fstream>
#include <iterator>
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "graphnav_mapper/mapper.hpp"
#include "graphnav_planner/planner.hpp"

template<typename T> T load(const std::string& path) {
  std::ifstream f(path,std::ios::binary);
  if(!f) throw std::runtime_error("Missing fixture "+path);
  std::vector<char> b((std::istreambuf_iterator<char>(f)),{});
  rclcpp::SerializedMessage s(b.size());
  auto& raw=s.get_rcl_serialized_message();
  std::memcpy(raw.buffer,b.data(),b.size());raw.buffer_length=b.size();
  T result;rclcpp::Serialization<T>().deserialize_message(&s,&result);return result;
}
template<typename T> void metadata(T& result,const auto& msg) {
  for(const auto& kv:msg.properties) result.metadata[kv.key]=kv.value;
  if(!msg.robot_traversability.empty())
    result.physical_observed_at_ns=rclcpp::Time(msg.robot_traversability[0].observed_at).nanoseconds();
}
int main(int argc,char** argv) {
  if(argc!=5) return 2;
  rclcpp::init(0,nullptr);
  auto raw=load<graphnav_msgs::msg::NavigationGraph>(argv[1]);
  auto terrain=std::make_shared<grid_map_msgs::msg::GridMap>(load<grid_map_msgs::msg::GridMap>(argv[2]));
  auto transform=load<geometry_msgs::msg::TransformStamped>(argv[3]);
  auto actual=std::make_shared<nav_msgs::msg::Odometry>(load<nav_msgs::msg::Odometry>(argv[4]));
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf(clock);tf.setTransform(transform,"recorded_fixture",true);
  auto logger=rclcpp::get_logger("exact_mapper_replay");logger.set_level(rclcpp::Logger::Level::Error);
  graphnav_mapper::Mapper mapper(tf,logger);
  mapper.robot_minimum_clearance_=1.06;mapper.traversability_update_nb_nodes_sample_=0;
  auto pg=std::make_shared<pose_graph_msgs::msg::PoseGraph>();
  pg->header=raw.header;pg->nodes.resize(1);pg->nodes[0].uuid.id[0]=1;
  pg->nodes[0].pose.orientation.w=1.;pg->current_node=pg->nodes[0].uuid;
  mapper.pose_graph_update(pg);
  auto proxy=std::make_shared<nav_msgs::msg::Odometry>(*actual);
  proxy->pose.pose=raw.nodes.at(raw.current_node_idx).pose;
  proxy->header.stamp=rclcpp::Time(rclcpp::Time(terrain->header.stamp).nanoseconds()-1000000000LL);
  mapper.odometry_update(proxy);
  auto& g=mapper.get_graph();auto first=g.get_all_nodes().front();
  std::vector<graphnav::NavigationGraph::UUID> ids;
  for(size_t i=0;i<raw.nodes.size();++i) {
    const auto& node=raw.nodes[i];graphnav::NavigationGraph::NodeProperties props;
    metadata(props,node);
    for(size_t c=0;c<raw.trav_classes.size() && c<node.trav_properties.size();++c) {
      const auto& source=node.trav_properties[c];auto& target=props.trav_properties[raw.trav_classes[c]];
      target.free_radius=source.free_radius;target.is_frontier=source.is_frontier;
      props.explored_radius=std::max(props.explored_radius,double(source.explored_radius));
      for(const auto& p:source.frontier_points) target.frontier_points.emplace_back(p.x,p.y,p.z);
      for(const auto& kv:source.properties) {
        target.metadata[kv.key]=kv.value;
        if(kv.key=="visited" && !kv.value.empty()) target.visited=kv.value[0]>.5f;
      }
      target.first_visited_time=proxy->header.stamp;
    }
    if(i==raw.current_node_idx) {g.set_node_properties(first,props);ids.push_back(first);}
    else ids.push_back(g.add_node(Eigen::Vector3d(node.pose.position.x,node.pose.position.y,node.pose.position.z),props));
  }
  for(const auto& edge:raw.edges) {
    graphnav::NavigationGraph::EdgeProperties props;metadata(props,edge);
    for(size_t c=0;c<raw.trav_classes.size() && c<edge.traversability.size();++c) {
      auto& target=props.trav_properties[raw.trav_classes[c]];
      target.cost=edge.traversability[c].traversability_cost;
      for(const auto& kv:edge.traversability[c].properties) {
        target.metadata[kv.key]=kv.value;
        if(kv.key=="traversed" && !kv.value.empty()) target.traversed=kv.value[0]>.5f;
      }
    }
    g.add_edge(ids.at(edge.from_idx),ids.at(edge.to_idx),props);
  }
  g.rebuild_kdtree();mapper.odometry_update(actual);
  auto audit=[&](const char* label) {
    auto msg=std::make_shared<graphnav_msgs::msg::NavigationGraph>(mapper.get_graph_msg(actual->header.stamp));
    graphnav_planner::Planner planner(logger);planner.set_trav_class("default");planner.update_graph(msg);
    int candidates=0,available=0;
    for(const auto& n:raw.nodes) {
      if(n.trav_properties.empty() || !n.trav_properties[0].is_frontier) continue;
      Eigen::Vector3d goal(n.pose.position.x,n.pose.position.y,n.pose.position.z);
      auto path=planner.plan_to_goal(goal,6.,rclcpp::Time(actual->header.stamp));
      ++candidates;if(!path.empty()) ++available;
    }
    const auto& node=msg->nodes.at(msg->current_node_idx);
    std::cout<<"REPLAY {\"phase\":\""<<label<<"\",\"nodes\":"<<msg->nodes.size()
      <<",\"available_frontier_routes\":"<<available<<",\"frontier_goals\":"<<candidates
      <<",\"current_xy\":["<<node.pose.position.x<<","<<node.pose.position.y<<"]}\\n"<<std::endl;
  };
  audit("actual_odom_rebound");
  mapper.traversability_update(terrain);
  audit("same_recorded_terrain_checked");
  rclcpp::shutdown();return 0;
}
