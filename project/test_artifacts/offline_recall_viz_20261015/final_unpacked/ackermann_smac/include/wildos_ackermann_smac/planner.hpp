// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <map>
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "wildos_ackermann_smac/connector.hpp"
namespace wildos_ackermann_smac {
class Planner : public nav2_smac_planner::SmacPlannerHybrid {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
private:
  std::mutex entry_mutex_;
  Limits limits_;
  double total_budget_ = 2., connector_budget_ = .2, resolution_ = 0;
  std::map<std::string, rclcpp::Parameter> frozen_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
};
}  // namespace wildos_ackermann_smac
