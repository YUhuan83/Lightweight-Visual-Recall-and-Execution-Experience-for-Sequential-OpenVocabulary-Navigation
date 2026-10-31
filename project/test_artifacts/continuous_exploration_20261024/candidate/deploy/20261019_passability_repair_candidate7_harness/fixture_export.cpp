// Export actual Mapper output, including its evidence timestamps, for DDS replay.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "graphnav_mapper/mapper.hpp"

int main(int argc,char** argv)
{
  if (argc!=2) return 2;
  const std::filesystem::path output(argv[1]);
  if (std::filesystem::exists(output)) return 3;
  std::filesystem::create_directories(output);
  rclcpp::init(argc,argv);
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  graphnav_mapper::Mapper mapper(buffer,rclcpp::get_logger("physical_fixture"));
  mapper.traversability_update_nb_nodes_sample_=0;
  mapper.traversability_classes_["default"]={"traversability",.001,1.};
  auto pose_graph=std::make_shared<pose_graph_msgs::msg::PoseGraph>();
  pose_graph->header.frame_id="map";pose_graph->header.stamp.sec=1;
  pose_graph->nodes.resize(1);pose_graph->nodes[0].uuid.id[0]=1;
  pose_graph->nodes[0].pose.orientation.w=1.;pose_graph->current_node=pose_graph->nodes[0].uuid;
  mapper.pose_graph_update(pose_graph);
  graphnav::NavigationGraph::NodeProperties properties;
  properties.trav_properties["default"].visited=true;
  properties.trav_properties["default"].free_radius=2.;
  auto& graph=mapper.get_graph();
  const auto a=graph.add_node(Eigen::Vector3d(2.,2.,0.),properties);
  const auto b=graph.add_node(Eigen::Vector3d(6.,2.,0.),properties);
  graphnav::NavigationGraph::EdgeProperties connection;
  connection.trav_properties["default"].cost=4.;connection.trav_properties["default"].traversed=true;
  graph.add_edge(a,b,connection);
  grid_map::GridMap map;
  map.setFrameId("map");map.setGeometry(grid_map::Length(50.,50.),.1,grid_map::Position(10.,5.));
  map.add("unknown",0.f);map.add("traversability",0.f);map.add("elevation",0.f);map.add("elevation_valid",1.f);
  map.add("witnessed_step",0.f);
  rclcpp::Serialization<graphnav_msgs::msg::NavigationGraph> serialization;
  const auto save=[&](const char* name,int seconds,bool update=true) {
    if (update) {
      map.setTimestamp(static_cast<uint64_t>(seconds)*1000000000ULL);
      mapper.traversability_update(grid_map::GridMapRosConverter::toMessage(map));
    }
    const auto message=graph.to_msg(rclcpp::Time(seconds,0,RCL_ROS_TIME),a);
    rclcpp::SerializedMessage data;serialization.serialize_message(&message,&data);
    const auto& raw=data.get_rcl_serialized_message();
    std::ofstream file(output/(std::string(name)+".cdr"),std::ios::binary);
    file.write(reinterpret_cast<const char*>(raw.buffer),raw.buffer_length);
    if (!file) throw std::runtime_error("Could not export serialized graph");
  };
  for (grid_map::GridMapIterator it(map);!it.isPastEnd();++it) {
    grid_map::Position p;map.getPosition(*it,p);
    map.at("elevation",*it)=.6*std::clamp(p.x()-3.,0.,.4);
    if (p.x()>5. && p.x()<5.3) map.at("elevation_valid",*it)=0.f;
    if (p.x()>5. && p.x()<5.5 && std::abs(p.y()-2.)<.5) map.at("unknown",*it)=1.f;
  }
  save("partial_steep",1);
  map["elevation_valid"].setZero();map["unknown"].setZero();save("unknown_after_steep",2);
  map["unknown"].setZero();map["elevation"].setZero();map["elevation_valid"].setOnes();
  save("flat_after_steep",3);
  map["elevation_valid"].setZero();save("unmeasured_flat",4);
  map["elevation_valid"].setOnes();
  for (grid_map::GridMapIterator it(map);!it.isPastEnd();++it) {
    grid_map::Position p;map.getPosition(*it,p);
    map.at("elevation",*it)=p.x()<4.?0.f:.6f;
    if (p.x()>3. && p.x()<5.) map.at("elevation_valid",*it)=0.f;
  }
  save("separated_flats",5);
  map["elevation_valid"].setOnes();
  for (grid_map::GridMapIterator it(map);!it.isPastEnd();++it) {
    grid_map::Position p;map.getPosition(*it,p);
    map.at("elevation",*it)=.6*std::clamp(p.x()-3.,0.,.4);
  }
  save("complete_steep",6);
  save("stale_steep",12,false);
  // A flat map with a measured sub-cell discontinuity. The elevation samples
  // are flat, so only the witnessed_step layer can reject this connection.
  map["elevation_valid"].setOnes();map["unknown"].setZero();
  map["witnessed_step"].setZero();
  for (grid_map::GridMapIterator it(map);!it.isPastEnd();++it) {
    grid_map::Position p;map.getPosition(*it,p);
    map.at("elevation",*it)=0.f;
    if (p.x()>3.9 && p.x()<4.1 && std::abs(p.y()-2.)<.3)
      map.at("witnessed_step",*it)=.17f;
  }
  save("witnessed_step_flat",13);
  // A measured low surface is represented as occupancy. Verify that the
  // occupancy -> SDF -> edge path rejects it even without a terrain slope.
  map["witnessed_step"].setZero();map["elevation"].setZero();
  map["elevation_valid"].setOnes();map["unknown"].setZero();
  map["traversability"].setZero();
  for (grid_map::GridMapIterator it(map);!it.isPastEnd();++it) {
    grid_map::Position p;map.getPosition(*it,p);
    if (p.x()>3.9 && p.x()<4.1 && std::abs(p.y()-2.)<.3)
      map.at("traversability",*it)=1.f;
  }
  save("low_surface_occupied",14);
  std::ifstream maps("/proc/self/maps");std::ofstream identity(output/"runtime_maps.txt");identity<<maps.rdbuf();
  rclcpp::shutdown();
  std::cout<<"Exported nine actual Mapper CDR graphs\n";
  return 0;
}
