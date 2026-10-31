#pragma once
#include <deque>
#include <memory>
#include <graphnav_msgs/msg/navigation_graph.hpp>

namespace graphnav_planner
{
// Python receives/scorers nodes only. Reattach edges only to the exact
// acquisition snapshot; indices from a newer graph must never be mixed in.
class GraphScoreOverlay
{
  using Graph = graphnav_msgs::msg::NavigationGraph;
  std::deque<Graph::ConstSharedPtr> snapshots_;
public:
  void remember(Graph::ConstSharedPtr graph)
  {
    snapshots_.push_back(std::move(graph));
    while (snapshots_.size() > 8) snapshots_.pop_front();
  }
  std::size_t size() const { return snapshots_.size(); }
  Graph::ConstSharedPtr combine(const Graph& scores) const
  {
    if (!scores.edges.empty()) return nullptr;
    for (auto it = snapshots_.rbegin(); it != snapshots_.rend(); ++it)
    {
      const auto& raw = **it;
      if (raw.header != scores.header || raw.trav_classes != scores.trav_classes ||
          raw.current_node_idx != scores.current_node_idx || raw.nodes.size() != scores.nodes.size()) continue;
      bool matches = true;
      for (std::size_t n = 0; n < raw.nodes.size(); ++n)
        if (raw.nodes[n].uuid != scores.nodes[n].uuid || raw.nodes[n].pose != scores.nodes[n].pose)
        { matches = false; break; }
      if (!matches) continue;
      auto combined = std::make_shared<Graph>(raw);
      combined->nodes = scores.nodes;
      return combined;
    }
    return nullptr;
  }
};
}
