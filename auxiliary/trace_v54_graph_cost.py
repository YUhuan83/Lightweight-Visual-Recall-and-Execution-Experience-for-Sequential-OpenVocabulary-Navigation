"""Generate a diagnostic-only production planner copy with read-only cost output."""
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/frontier_endpoint_contract_review')
p=root/'candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner.cpp'
s=p.read_text()
needle='  auto path = graaf::algorithm::dijkstra_shortest_path(graph_, current_node_idx_, virtual_goal);'
trace='''
  if (std::getenv("TRACE_PLAN")) {
    for (const auto& [id, scored] : frontier_scores_) {
      auto route = graaf::algorithm::dijkstra_shortest_path(graph_, current_node_idx_, id);
      double travel = 0.;
      if (route && !route->vertices.empty()) {
        auto previous=route->vertices.begin();
        for (auto current=std::next(previous);current!=route->vertices.end();++current,++previous)
          travel += graph_.get_edge(*previous,*current);
      }
      const auto& pos=scored.first.pose.position;
      std::cout<<"TRACE "<<id<<" "<<pos.x<<" "<<pos.y<<" reachable="<<bool(route)
        <<" travel="<<travel<<" tail="<<scored.second.second
        <<" total="<<(travel+scored.second.second)<<std::endl;
    }
  }
'''
assert s.count(needle)==1
(root/'trace_planner.cpp').write_text(s.replace(needle,needle+trace))
lines=[line for line in (root/'recorded_graphs.txt').read_text().splitlines() if 'pair_000156_' in line]
assert len(lines)==1
(root/'trace_graph.txt').write_text(lines[0]+'\n')
cmake=root/'CMakeLists.txt'
t=cmake.read_text()
if 'add_executable(replay_trace' not in t:
 t+='''
add_executable(replay_trace trace_planner.cpp replay_graph.cpp)
target_include_directories(replay_trace PRIVATE "${PLANNER_ROOT}/include" ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(replay_trace rclcpp graphnav_msgs visualization_msgs grid_map_msgs nav_msgs geometry_msgs graaf_vendor)
'''
 cmake.write_text(t)
