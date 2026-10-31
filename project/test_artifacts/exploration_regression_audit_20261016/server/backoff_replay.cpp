// Diagnostic only. Replay frozen, closed observations with production classes.
#include <fstream>
#include <iostream>
#include <iterator>
#include <rclcpp/serialization.hpp>
#include <graphnav_msgs/msg/navigation_graph.hpp>
#include "graphnav_planner/exploration_backoff.hpp"
#include "graphnav_planner/planner.hpp"

int main() {
  using graphnav_planner::ExplorationBackoff;
  ExplorationBackoff backoff;
  char op;
  while (std::cin >> op) {
    if (op == 'C') { backoff.clear(); continue; }
    int64_t now; std::cin >> now;
    if (op == 'R') {
      int64_t stamp; size_t count;
      geometry_msgs::msg::PoseStamped msg;
      geometry_msgs::msg::Point dest, failure;
      std::cin >> stamp >> msg.pose.position.x >> msg.pose.position.y
               >> dest.x >> dest.y >> failure.x >> failure.y >> count;
      msg.header.frame_id = "map";
      msg.header.stamp = rclcpp::Time(stamp);
      std::vector<geometry_msgs::msg::Point> path(count);
      for (auto& p : path) std::cin >> p.x >> p.y;
      backoff.remember(msg.header, msg.pose.position, dest, "recorded", path);
      std::cout << "R " << now << " " << backoff.reject(msg, now, true, failure) << "\n";
    } else if (op == 'Q') {
      size_t count; std::cin >> count;
      std::cout << "Q " << now;
      for (size_t i=0; i<count; ++i) {
        geometry_msgs::msg::Point p; std::cin >> p.x >> p.y;
        std::cout << " " << backoff.blocked(p,now,3.);
      }
      std::cout << "\n";
    } else if (op == 'P') {
      std::string filename; size_t count;
      std::cin >> filename >> count;
      std::ifstream file(filename, std::ios::binary);
      std::vector<char> bytes((std::istreambuf_iterator<char>(file)), {});
      if (bytes.empty()) return 2;
      rclcpp::SerializedMessage serialized(bytes.size());
      auto& raw = serialized.get_rcl_serialized_message();
      std::copy(bytes.begin(), bytes.end(), raw.buffer);
      raw.buffer_length = bytes.size();
      auto graph = std::make_shared<graphnav_msgs::msg::NavigationGraph>();
      rclcpp::Serialization<graphnav_msgs::msg::NavigationGraph> codec;
      codec.deserialize_message(&serialized, graph.get());
      graphnav_planner::Planner unrestricted(rclcpp::get_logger("audit"));
      graphnav_planner::Planner restricted(rclcpp::get_logger("audit"));
      unrestricted.set_trav_class("default"); restricted.set_trav_class("default");
      unrestricted.update_graph(graph);
      restricted.set_rejected_edges(backoff.active_edges(now));
      restricted.update_graph(graph);
      std::cout << "P " << now << " " << backoff.active_edges(now).size();
      for (size_t i=0; i<count; ++i) {
        size_t index; std::cin >> index;
        auto p = graph->nodes.at(index).pose.position;
        Eigen::Vector3d goal(p.x,p.y,p.z);
        auto a = unrestricted; auto b = restricted;
        const auto ap = a.plan_to_goal(goal,3.,rclcpp::Time(now));
        const auto bp = b.plan_to_goal(goal,3.,rclcpp::Time(now));
        std::cout << " " << index << ":" << backoff.blocked(p,now,3.)
                  << ":" << ap.size() << ":" << a.route_complete()
                  << ":" << bp.size() << ":" << b.route_complete();
      }
      std::cout << "\n";
    } else return 3;
  }
}
