from pathlib import Path
ROOT=Path(r'C:\Users\Yu\Desktop\拟真')
BASE=ROOT/'wildos_port/repro_check/src/nebula2-wildos'
def edit(path,old,new,count=1):
 p=Path(path); s=p.read_text(encoding='utf-8')
 assert s.count(old)>=count,(str(p),old[:100])
 p.write_text(s.replace(old,new,count),encoding='utf-8')
# Handoff keeps the selected graph route and local path, including its actual endpoint.
p=ROOT/'nav2_adapter/wildos_nav2_adapter/memory_selection.py'
edit(p,'complete=self.memory_full_check, path=deepcopy(path)))',
'''complete=self.memory_full_check, path=deepcopy(path),
            route=deepcopy(self.route), route_complete=getattr(self, 'route_complete', True),
            target=tuple(points[-1])))''')
edit(p,"'outcome': 'unreachable' if reason in ('no_vehicle_path', 'follow_failed') else 'unavailable',",
       "'outcome': 'unavailable',")
edit(p,"        return self.reject_selected_memory('no_vehicle_path')",
'''        if self.memory_active is not None:
            self.block_current_route()
            self.status('memory_locally_blocked', reason='no_vehicle_path',
                        goal=list(self.memory_goal), reasons=list(self.failure_reasons))
            return True
        return False''')
edit(p,"""        else:
            self.relay_destination(self.make_memory_pose(best['view'], best['heading']))
            self.status('memory_candidate_selected', view_id=best['view']['id'], cost=best['cost'])
""","""        else:
            self.relay_destination(self.make_memory_pose(best['view'], best['heading']))
            self.status('memory_candidate_selected', view_id=best['view']['id'], cost=best['cost'])
            if best['path'] is not None:
                self.route = deepcopy(best['route'])
                self.latest_route = self.route
                self.route_complete = best.get('route_complete', True)
                self.route_endpoint = position(self.route.poses[-1].pose)
                self.execute_checked_path(deepcopy(best['path']), best['target'])
""")
# Vehicle odometry and geometry-aware lookahead.
p=ROOT/'nav2_adapter/config/nav2_official_control.yaml'
edit(p,'    controller_frequency: 10.0','    odom_topic: /spot1/odom\n    controller_frequency: 10.0')
edit(p,'      min_lookahead_dist: 1.5','      min_lookahead_dist: 3.0')
edit(p,'      max_lookahead_dist: 5.0','      max_lookahead_dist: 6.0')
edit(p,'      lookahead_time: 1.5','      lookahead_time: 2.5')
edit(p,'      regulated_linear_scaling_min_radius: 1.5','      regulated_linear_scaling_min_radius: 8.0')
# Typed assessment age rather than publication time.
p=BASE/'graphnav_mapper/include/graphnav_mapper/navigation_graph.hpp'
edit(p,'    double explored_radius = 0.0;\n    Metadata metadata;',
       '    double explored_radius = 0.0;\n    uint64_t physical_observed_at_ns = 0;\n    Metadata metadata;')
edit(p,'    std::map<std::string, trav_properties_t> trav_properties;\n    Metadata metadata;',
       '    std::map<std::string, trav_properties_t> trav_properties;\n    uint64_t physical_observed_at_ns = 0;\n    Metadata metadata;')
p=BASE/'graphnav_mapper/src/mapper.cpp'
edit(p,'    properties.trav_properties[my_trav_class_].traversed = true;',
'''    properties.trav_properties[my_trav_class_].traversed = true;
    properties.trav_properties[my_trav_class_].cost =
        (graph_.get_node_position(*current_node_) - graph_.get_node_position(*closest_node)).norm();''')
edit(p,'    graphnav::NavigationGraph::EdgeProperties edge_properties;  // TODO: add traversability',
'''    graphnav::NavigationGraph::EdgeProperties edge_properties;
    edge_properties.trav_properties[my_trav_class_].traversed = true;
    edge_properties.trav_properties[my_trav_class_].cost =
        (graph_.get_node_position(*current_node_) - odom_position).norm();''')
# No height source in occupancy-grid adapter: retain z=0 for graph geometry only.
p=BASE/'graphnav_mapper/src/mapper_node.cpp'
edit(p,'grid_map->layers = { "traversability", "unknown", "elevation" };',
       'grid_map->layers = { "traversability", "unknown", "elevation", "elevation_valid" };')
edit(p,'grid_map->data.resize(3);','grid_map->data.resize(4);')
edit(p,'  grid_map->data[2].layout = grid_map->data[0].layout;',
       '  grid_map->data[2].layout = grid_map->data[0].layout;\n  grid_map->data[3].layout = grid_map->data[0].layout;')
edit(p,'  grid_map->data[2].data.resize(size_x * size_y);',
       '  grid_map->data[2].data.resize(size_x * size_y);\n  grid_map->data[3].data.assign(size_x * size_y, 0.0f);')
p=BASE/'graphnav_mapper/src/mapper.cpp'
edit(p,'  layer_names.push_back("elevation");',
       '  layer_names.push_back("elevation");\n  if (gridmap.exists("elevation_valid")) layer_names.push_back("elevation_valid");')
edit(p,'  double edge_min_clearance = std::numeric_limits<double>::infinity();',
'''  double edge_min_clearance = std::numeric_limits<double>::infinity();
  bool fully_observed = true;
  bool terrain_observed = !map.exists("elevation_valid");''')
edit(p,'      const double z = map.atPosition("elevation", p);',
'''      fully_observed = fully_observed && map.atPosition("unknown", p) < 0.5f;
      const double z = terrain_observed ? map.atPosition("elevation", p) :
          std::numeric_limits<double>::quiet_NaN();''')
edit(p,'  properties.metadata["physical_feasible"] = {physical_edge_feasible ? 1.0f : 0.0f};',
'''  properties.physical_observed_at_ns = map.getTimestamp();
  properties.metadata["physical_confidence"] = {fully_observed ? 0.8f : 0.0f};
  properties.metadata["terrain_observed"] = {terrain_observed ? 1.0f : 0.0f};
  properties.metadata["physical_feasible"] = {physical_edge_feasible ? 1.0f : 0.0f};''')
edit(p,'      if (gridmap.exists("elevation"))',
'''      if (gridmap.exists("elevation") && !gridmap.exists("elevation_valid"))''')
# Occupancy only assessments: independently measure 2D clearance and flag missing terrain.
edit(p,'      if (node_prop.trav_properties.empty())',
'''      if (gridmap.exists("elevation_valid"))
      {
        const double clearance = gridmap.atPosition("traversability_sdf", node_pos_map);
        node_prop.physical_observed_at_ns = gridmap.getTimestamp();
        node_prop.metadata["terrain_observed"] = {0.0f};
        node_prop.metadata["minimum_clearance"] = {static_cast<float>(clearance)};
        node_prop.metadata["physical_confidence"] = {unknown_dist > robot_minimum_clearance_ ? 0.8f : 0.0f};
        node_prop.metadata["physical_feasible"] = {clearance >= robot_minimum_clearance_ ? 1.0f : 0.0f};
        node_prop_changed = true;
      }
      if (node_prop.trav_properties.empty())''')
edit(p,'          node_prop.metadata["physical_risk"] = {static_cast<float>(physical_risk)};',
'''          node_prop.physical_observed_at_ns = gridmap.getTimestamp();
          node_prop.metadata["physical_confidence"] = {unknown_dist > robot_minimum_clearance_ ? 0.8f : 0.0f};
          node_prop.metadata["terrain_observed"] = {1.0f};
          node_prop.metadata["physical_risk"] = {static_cast<float>(physical_risk)};''')
p=BASE/'graphnav_mapper/src/navigation_graph.cpp'
edit(p,'    robot_node.confidence = node.properties.metadata.count("slope") ? 0.8f : 0.0f;\n    robot_node.observed_at = stamp;',
'''    robot_node.confidence = read_node_value("physical_confidence", 0.0f);
    robot_node.observed_at = rclcpp::Time(static_cast<int64_t>(node.properties.physical_observed_at_ns)).to_msg();''')
edit(p,'      edge_msg.traversability[trav_class_idx].traversability_cost = trav_prop.cost;',
'''      edge_msg.traversability[trav_class_idx].traversability_cost = trav_prop.cost;
      graphnav_msgs::msg::KeyValue traversed;
      traversed.key = "traversed";
      traversed.value = {trav_prop.traversed ? 1.0f : 0.0f};
      edge_msg.traversability[trav_class_idx].properties.push_back(traversed);''')
edit(p,'    robot_edge.confidence = robot_edge.physical_traversability;\n    robot_edge.observed_at = stamp;',
'''    robot_edge.confidence = read_edge_value("physical_confidence", 0.0f);
    robot_edge.observed_at = rclcpp::Time(static_cast<int64_t>(edge.properties.physical_observed_at_ns)).to_msg();''')
# Planner honors traversed topology prior; current Nav2 footprint validation remains mandatory.
p=BASE/'graphnav_planner/src/planner.cpp'
a=p.read_text().index('    const bool endpoints_feasible =')
b=p.read_text().index('      double weight =',a)
s=p.read_text(); s=s[:a]+'''    if (edge.from_idx >= graph->nodes.size() || edge.to_idx >= graph->nodes.size() ||
        trav_class_idx_ >= edge.traversability.size()) continue;
    bool traversed = false;
    for (const auto& property : edge.traversability[trav_class_idx_].properties)
      if (property.key == "traversed" && !property.value.empty() && property.value.front() > 0.5f)
        traversed = true;
    const auto* edge_assessment = profile_edge_assessment(edge, robot_profile_id_);
    const auto fresh = [&](const auto* assessment) {
      if (assessment == nullptr || assessment->observed_at.sec == 0 ||
          assessment->confidence < 0.7f) return false;
      const double age = (rclcpp::Time(graph->header.stamp) -
                          rclcpp::Time(assessment->observed_at)).seconds();
      return age >= 0.0 && age <= stale_observation_seconds_;
    };
    bool endpoints_feasible = true;
    for (const auto index : {edge.from_idx, edge.to_idx}) {
      const auto* assessment = profile_node_assessment(graph->nodes[index], robot_profile_id_);
      if (fresh(assessment) && !assessment->feasible) endpoints_feasible = false;
    }
    const bool fresh_dynamic_assessment = fresh(edge_assessment);
    const bool hard_blocked = fresh_dynamic_assessment &&
        edge_assessment->physical_traversability <= 0.0f;
    if (traversed || (endpoints_feasible && !hard_blocked))
    {
'''+s[b:];p.write_text(s)
edit(p,'      graph_.add_edge(edge.from_idx, edge.to_idx, weight);',
'''      if (traversed && (!endpoints_feasible || hard_blocked)) weight += physical_risk_factor_;
      if (!std::isfinite(weight) || weight < 0.0) continue;
      graph_.add_edge(edge.from_idx, edge.to_idx, weight);''')
# Known connectivity first. Frontier heuristic edges are only fallbacks.
edit(p,'  if (!unexplored_space_map_)\n  {',
       '  route_complete_ = false;\n  if (!unexplored_space_map_)\n  {')
edit(p,'    if (!profile_node_feasible(node, robot_profile_id_))',
       '    if (id == virtual_goal) continue;\n    if (!profile_node_feasible(node, robot_profile_id_))')
edit(p,'      Eigen::Vector3d heading = (goal - node_pos).normalized();',
       '      Eigen::Vector3d heading = goal - node_pos;')
edit(p,'      frontier_scores_[id] = std::make_pair',
       '      if (!std::isfinite(frontier_cost)) continue;\n      frontier_cost = std::max(0.0, frontier_cost);\n      frontier_scores_[id] = std::make_pair')
edit(p,'  // if current_time is within path_smoothness_period_',
'''  auto known_path = graaf::algorithm::dijkstra_shortest_path(graph_, current_node_idx_, virtual_goal);
  if (known_path)
  {
    std::vector<Eigen::Vector3d> points;
    for (const auto id : known_path->vertices) {
      const auto& pos = graph_.get_vertex(id).pose.position;
      points.emplace_back(pos.x, pos.y, pos.z);
    }
    route_complete_ = true;
    latest_frontier_.reset();
    latest_frontier_time_.reset();
    graph_.remove_vertex(virtual_goal);
    return points;
  }
  // if current_time is within path_smoothness_period_''')
edit(p,'    if ((current_time - *latest_frontier_time_).seconds()',
       '    if (latest_frontier_time_ && (current_time - *latest_frontier_time_).seconds()')
edit(p,'      const auto& node = graph_.get_vertex(node_id);',
       '      if (node_id == virtual_goal) continue; // Estimated unseen tail is not an executable route.\n      const auto& node = graph_.get_vertex(node_id);')
edit(p,'  if (latest_frontier_time_)\n  {','  if (latest_frontier_time_ && latest_frontier_)\n  {')
p=BASE/'graphnav_planner/include/graphnav_planner/planner.hpp'
edit(p,'  std::vector<Eigen::Vector3d> plan_to_goal',
       '  bool route_complete() const { return route_complete_; }\n  bool route_complete_ = false;\n  std::vector<Eigen::Vector3d> plan_to_goal')
p=BASE/'graphnav_planner/src/planner_node.cpp'
edit(p,'#include <std_msgs/msg/header.hpp>','#include <std_msgs/msg/header.hpp>\n#include <std_msgs/msg/string.hpp>')
edit(p,'    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);',
'''    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    route_status_pub_ = this->create_publisher<std_msgs::msg::String>("~/route_status", 10);''')
s=p.read_text(); a=s.index('        if (i < path.size() - 1)'); b=s.index('      path_pub_->publish(path_msg);',a)
s=s[:a]+'''        double heading = i ? 2.0 * std::atan2(path_msg.poses[i-1].pose.orientation.z,
                                                path_msg.poses[i-1].pose.orientation.w) : 0.0;
        if (i + 1 < path.size() && (path[i+1].head<2>() - path[i].head<2>()).norm() > 1e-6)
          heading = std::atan2(path[i+1].y() - path[i].y(), path[i+1].x() - path[i].x());
        path_msg.poses[i].header = path_msg.header;
        path_msg.poses[i].pose.orientation.z = std::sin(heading / 2.0);
        path_msg.poses[i].pose.orientation.w = std::cos(heading / 2.0);
      }
      std_msgs::msg::String status;
      std::ostringstream encoded;
      encoded << std::setprecision(17) << "{\\"stamp_ns\\":"
              << rclcpp::Time(path_msg.header.stamp).nanoseconds()
              << ",\\"goal\\":[" << goal_vec.x() << "," << goal_vec.y()
              << "],\\"complete\\":" << (planner_.route_complete() ? "true" : "false")
              << ",\\"mode\\":\\"" << (path.empty() ? "none" : planner_.route_complete() ? "known" : "frontier")
              << "\\"}";
      status.data = encoded.str();
      route_status_pub_->publish(status);
'''+s[b:];p.write_text(s)
edit(p,'  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;',
       '  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;\n  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_status_pub_;')
print('patched mapper/planner/memory/controller')

