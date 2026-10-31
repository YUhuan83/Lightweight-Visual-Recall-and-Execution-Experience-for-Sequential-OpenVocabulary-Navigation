// SPDX-License-Identifier: Apache-2.0
#include "wildos_ackermann_smac/planner.hpp"
#include "wildos_ackermann_smac/path_contract.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wildos_ackermann_smac {
static Pose poseOf(const geometry_msgs::msg::PoseStamped & p) {
  return {p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)};
}
static geometry_msgs::msg::PoseStamped stamped(Pose p, const std_msgs::msg::Header & header) {
  geometry_msgs::msg::PoseStamped out; out.header = header;
  out.pose.position.x = p.x; out.pose.position.y = p.y;
  tf2::Quaternion q; q.setRPY(0, 0, p.yaw); out.pose.orientation = tf2::toMsg(q); return out;
}
void Planner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  SmacPlannerHybrid::configure(parent, name, tf, costmap);
  auto node = parent.lock();
  auto declare = [&](const std::string & key, rclcpp::ParameterValue value) {
    nav2_util::declare_parameter_if_not_declared(node, name + "." + key, value);
  };
  declare("max_total_planning_time", rclcpp::ParameterValue(2.));
  declare("connector_search_time", rclcpp::ParameterValue(.2));
  declare("connector_max_evaluations", rclcpp::ParameterValue(6000));
  declare("connector_max_candidates", rclcpp::ParameterValue(6));
  declare("connector_min_section", rclcpp::ParameterValue(.4));
  node->get_parameter(name + ".max_total_planning_time", total_budget_);
  node->get_parameter(name + ".connector_search_time", connector_budget_);
  int evaluations, candidates;
  node->get_parameter(name + ".connector_max_evaluations", evaluations);
  node->get_parameter(name + ".connector_max_candidates", candidates);
  node->get_parameter(name + ".connector_min_section", limits_.min_section);
  if (!std::isfinite(total_budget_) || !std::isfinite(connector_budget_) ||
    !std::isfinite(_max_planning_time) || _max_planning_time <= 0 ||
    connector_budget_ <= 0 || total_budget_ < _max_planning_time + connector_budget_ + .05 ||
    evaluations < 1 || evaluations > 100000 || candidates < 1 || candidates > 64 ||
    !std::isfinite(limits_.min_section) || limits_.min_section <= 0 ||
    _motion_model_for_search != "REEDS_SHEPP" || _downsample_costmap ||
    _costmap_ros->getUseRadius()) {
    throw std::runtime_error("Continuous start requires frozen polygon/REEDS_SHEPP, no downsample, valid budgets");
  }
  limits_.max_evaluations = evaluations; limits_.max_candidates = candidates;
  limits_.radius = _minimum_turning_radius_global_coords;
  limits_.bins = _angle_quantizations; limits_.allow_unknown = _allow_unknown;
  resolution_ = _costmap->getResolution();
  frozen_.clear();
  for (const auto & key : node->list_parameters({name}, 100).names) {
    frozen_.emplace(key, node->get_parameter(key));
  }
}
void Planner::activate() {
  auto node = _node.lock();
  for (const auto & item : frozen_) {
    if (node->get_parameter(item.first).get_parameter_value() != item.second.get_parameter_value()) {
      throw std::runtime_error("Planner parameters changed while inactive; cleanup/configure required");
    }
  }
  SmacPlannerHybrid::activate();
  // The base callback mutates backend state before other callbacks may reject changes.
  // Remove it; freeze this planner's complete parameter namespace while active.
  _dyn_params_handler.reset();
  guard_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result; result.successful = true;
    for (const auto & p : params) {
      if (p.get_name().rfind(_name + ".", 0) != 0) { continue; }
      auto found = frozen_.find(p.get_name());
      if (found == frozen_.end() ||
        p.get_parameter_value() != found->second.get_parameter_value()) {
        result.successful = false; result.reason = "Planner parameters frozen; deactivate/reconfigure";
        break;
      }
    }
    return result;
  });
}
void Planner::deactivate() { guard_.reset(); SmacPlannerHybrid::deactivate(); }
void Planner::cleanup() {
  std::lock_guard<std::mutex> entry(entry_mutex_);
  guard_.reset(); frozen_.clear(); SmacPlannerHybrid::cleanup();
}
nav_msgs::msg::Path Planner::createPlan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto began = Clock::now();
  const auto deadline = began + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(total_budget_));
  std::lock_guard<std::mutex> entry(entry_mutex_);
  if (start.header.frame_id != _global_frame || goal.header.frame_id != _global_frame) {
    throw nav2_core::PlannerException("Continuous start frame mismatch");
  }
  for (const auto * p : {&start.pose, &goal.pose}) {
    const auto & q = p->orientation;
    double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(p->position.x) || !std::isfinite(p->position.y) ||
      !std::isfinite(p->position.z) || !std::isfinite(norm) || std::abs(norm - 1.) > 1e-3) {
      throw nav2_core::PlannerException("Invalid continuous start/goal pose");
    }
  }
  auto actual = poseOf(start);
  SearchResult candidates; nav2_costmap_2d::Footprint footprint;
  bool ordinary;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    if (_costmap->getResolution() != resolution_ || _costmap_ros->getUseRadius()) {
      throw nav2_core::PlannerException("Costmap geometry changed; reconfigure planner");
    }
    footprint = _costmap_ros->getRobotFootprint();
    if (!safe(*_costmap, footprint, actual, _allow_unknown)) {
      throw nav2_core::PlannerException("Actual continuous start is in collision");
    }
    unsigned x, y; _costmap->worldToMap(actual.x, actual.y, x, y);
    int bin = static_cast<int>(std::round(actual.yaw / _angle_bin_size));
    bin = (bin % static_cast<int>(_angle_quantizations) + _angle_quantizations) % _angle_quantizations;
    nav2_smac_planner::GridCollisionChecker checker(_costmap, _angle_quantizations, nullptr);
    checker.setFootprint(footprint, false, 0);
    double lattice_x,lattice_y;
    _costmap->mapToWorld(x,y,lattice_x,lattice_y);
    const double lattice_yaw=bin*_angle_bin_size;
    // A clear quantized footprint is not a continuous-start contract. Native
    // Hybrid A* returns the lattice center/heading; starting from a different
    // real axle pose can make a curvature-limited follower miss the next cusp.
    // Only bypass the connector when the actual request already is that state.
    ordinary = std::hypot(actual.x-lattice_x,actual.y-lattice_y)<=1e-6 &&
      std::abs(std::remainder(actual.yaw-lattice_yaw,2.*std::acos(-1.)))<=1e-5 &&
      !checker.inCollision(x, y, bin, _allow_unknown);
    if (!ordinary) {
      const auto search_deadline = std::min(deadline, Clock::now() +
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(connector_budget_)));
      candidates = search(*_costmap, footprint, actual, limits_, search_deadline);
      RCLCPP_INFO(_logger, "continuous_start: evaluated=%u candidates=%zu timeout=%d",
        candidates.evaluated, candidates.curves.size(), candidates.timed_out);
    }
  }  // Never hold map mutex while invoking the native planner (lock order).
  auto validate = [&](nav_msgs::msg::Path & path) {
    if (Clock::now() >= deadline || path.poses.empty()) { return false; }
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    if (_costmap->getResolution() != resolution_) { return false; }
    auto current_body = _costmap_ros->getRobotFootprint();
    for (const auto & p : path.poses) {
      if (Clock::now() >= deadline || !safe(*_costmap, current_body, poseOf(p), _allow_unknown)) {
        return false;
      }
    }
    return true;
  };
  auto enough_time = [&](bool optional = false) {
    // Preserve first-search admission; reserve more time to recheck the retained
    // full route when spending spare budget on another alternative.
    return std::chrono::duration<double>(deadline - Clock::now()).count() >=
      _max_planning_time + (optional ? .05 : .01);
  };
  auto native_plan = [&](const geometry_msgs::msg::PoseStamped & native_start) {
    // Keep the raw result of this exact search. The native smoother modifies
    // poses even when it reports failure and does not validate curvature.
    // Never pay for a second search merely to recover the original path.
    const auto native_began=Clock::now();
    auto smoother=std::move(_smoother);
    nav_msgs::msg::Path raw;
    try { raw=SmacPlannerHybrid::createPlan(native_start,goal); }
    catch(...) { _smoother=std::move(smoother);throw; }
    _smoother=std::move(smoother);
    if(raw.poses.empty()) { return raw; }
    auto geometry_ok = [&](const nav_msgs::msg::Path & path) {
      std::vector<Pose> poses;poses.reserve(path.poses.size());
      for(const auto & p:path.poses) { poses.push_back(poseOf(p)); }
      return curvatureFeasible(poses,resolution_,limits_.radius,limits_.bins);
    };
    const double remaining=std::min(
      _max_planning_time-std::chrono::duration<double>(Clock::now()-native_began).count(),
      std::chrono::duration<double>(deadline-Clock::now()).count());
    if(_smoother && remaining>0) {
      auto smooth=raw;bool successful=false;
      {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
        if(_costmap->getResolution()!=resolution_) {
          throw nav2_core::PlannerException("Costmap geometry changed during smoothing");
        }
        successful=_smoother->smooth(smooth,_costmap,remaining);
      }
      if(successful && geometry_ok(smooth) && validate(smooth)) { return smooth; }
      RCLCPP_INFO(_logger,"smoothed path rejected; validating same-search raw path");
    }
    if(!geometry_ok(raw) || !validate(raw)) {
      throw nav2_core::PlannerException("Raw path violates directional geometry, collision or time budget");
    }
    return raw;
  };
  if (ordinary) {
    if (!enough_time()) { throw nav2_core::PlannerException("Planning budget exhausted before native search"); }
    auto path = native_plan(start);
    if (!validate(path)) { throw nav2_core::PlannerException("Native path expired or collides with current map"); }
    return path;
  }
  // A short start connector can lead to an expensive suffix with many cusps.
  // Compare feasible complete routes, bounded by the existing candidate/time caps.
  std::vector<std::pair<double, nav_msgs::msg::Path>> feasible;
  for (const auto & c : candidates.curves) {
    if (!enough_time(!feasible.empty())) { break; }
    auto anchor = stamped(c.poses.back(), start.header);
    nav_msgs::msg::Path suffix;
    try { suffix = native_plan(anchor); }
    catch (const std::runtime_error &) { continue; }
    if (suffix.poses.empty()) { continue; }
    auto first = poseOf(suffix.poses.front()), end = c.poses.back();
    if (std::hypot(first.x - end.x, first.y - end.y) > 1e-6 ||
      std::abs(std::atan2(std::sin(first.yaw - end.yaw), std::cos(first.yaw - end.yaw))) > 1e-5) {
      continue;
    }
    nav_msgs::msg::Path result; result.header = suffix.header;
    for (const auto & p : c.poses) { result.poses.push_back(stamped(p, result.header)); }
    result.poses.insert(result.poses.end(), suffix.poses.begin() + 1, suffix.poses.end());
    std::vector<Pose> combined;combined.reserve(result.poses.size());
    for(const auto & p:result.poses) { combined.push_back(poseOf(p)); }
    if (curvatureFeasible(combined,resolution_,limits_.radius,limits_.bins) && validate(result)) {
      const double cost = maneuverCost(combined);
      if (std::isfinite(cost)) { feasible.emplace_back(cost, std::move(result)); }
    }
  }
  std::stable_sort(feasible.begin(), feasible.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });
  // Maps may change during subsequent searches. Never return a cached safety
  // verdict; fall back to the next alternative if the cheapest is now blocked.
  for (auto & candidate : feasible) {
    if (validate(candidate.second)) {
      RCLCPP_INFO(_logger, "continuous_complete_selected: cost=%.6f candidates=%zu",
        candidate.first, feasible.size());
      return candidate.second;
    }
  }
  throw nav2_core::PlannerException("No safe bounded continuous-start plan");
}
}  // namespace wildos_ackermann_smac
PLUGINLIB_EXPORT_CLASS(wildos_ackermann_smac::Planner, nav2_core::GlobalPlanner)
