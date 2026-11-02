// SPDX-License-Identifier: Apache-2.0
#include "wildos_ackermann_smac/planner.hpp"
#include "wildos_ackermann_smac/path_contract.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
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
  // 0 keeps the historic ranking. A positive value adds this cost to a
  // candidate whose first edge points away from the goal, so an equally safe
  // forward-first connector wins (205052: 26/40 selections were reverse-first).
  declare("reverse_start_penalty", rclcpp::ParameterValue(0.));
  node->get_parameter(name + ".reverse_start_penalty", reverse_start_penalty_);
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
    !std::isfinite(reverse_start_penalty_) || reverse_start_penalty_ < 0 ||
    reverse_start_penalty_ > 100 ||
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
      // The native planner restores endpoint orientations after its smoother.
      // Here smoothing is separate so the raw result can be retained. Do not
      // choose a smooth suffix whose boundary poses no longer join its exact
      // connector or preserve the searched endpoint; keep the same-search raw
      // alternative available for the existing geometry and collision checks.
      auto same_pose = [](const auto & a, const auto & b) {
        const auto pa = poseOf(a), pb = poseOf(b);
        return std::hypot(pa.x-pb.x,pa.y-pb.y)<=1e-6 &&
          std::abs(std::remainder(pa.yaw-pb.yaw,2.*std::acos(-1.)))<=1e-5;
      };
      if(successful && !smooth.poses.empty() &&
        same_pose(smooth.poses.front(),raw.poses.front()) &&
        same_pose(smooth.poses.back(),raw.poses.back()) &&
        geometry_ok(smooth) && validate(smooth)) { return smooth; }
      RCLCPP_INFO(_logger,"smoothed path rejected; validating same-search raw path");
    }
    if(!geometry_ok(raw) || !validate(raw)) {
      throw nav2_core::PlannerException("Raw path violates directional geometry, collision or time budget");
    }
    return raw;
  };
  // A short task need not visit an intermediate lattice state. Retain the
  // exact same bounded Reeds-Shepp connection used for start anchors as a
  // complete goal candidate; never replace the obstacle-avoiding search.
  nav_msgs::msg::Path direct; direct.header=start.header;
  // The connector used for lattice anchors stays capped at one turning radius.
  // A complete goal connection has a separate existing Smac bound; otherwise
  // a long straight route inherits the short anchor cap and falls back to a
  // lattice route with unnecessary gear changes.
  const auto goal_curve=connect(actual,poseOf(goal),limits_.radius,limits_.min_section,
    std::min(.05,resolution_/4),_search_info.analytic_expansion_max_length);
  if (!goal_curve.poses.empty() &&
    curvatureFeasible(goal_curve.poses,resolution_,limits_.radius,limits_.bins)) {
    for (const auto & p:goal_curve.poses) { direct.poses.push_back(stamped(p,direct.header)); }
    if (!validate(direct)) { direct.poses.clear(); }
  }
  // Diagnostic only (no behavior change): attribute reverse-first sections to
  // the direct connection, the continuous-start connector or the Smac suffix.
  // trial_20261021_183339 drove north for ~75 s with a goal ~25 m south; the
  // recorder now also keeps /wildos/full_planned_path for the same split.
  auto cusp_count = [](const nav_msgs::msg::Path & path) {
    size_t count = 0; bool have = false; double px = 0., py = 0.;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto a = poseOf(path.poses[i-1]), b = poseOf(path.poses[i]);
      const double dx = b.x - a.x, dy = b.y - a.y;
      if (std::hypot(dx, dy) <= 1e-9) { continue; }
      if (have && dx*px + dy*py < 0.) { ++count; }
      px = dx; py = dy; have = true;
    }
    return count;
  };
  auto first_cosine = [&](const nav_msgs::msg::Path & path) {
    if (path.poses.empty()) { return 1.; }
    const auto start = poseOf(path.poses.front());
    const double gx = goal.pose.position.x - start.x, gy = goal.pose.position.y - start.y;
    const double norm = std::hypot(gx, gy);
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto a = poseOf(path.poses[i-1]), b = poseOf(path.poses[i]);
      const double dx = b.x - a.x, dy = b.y - a.y, length = std::hypot(dx, dy);
      if (length <= 1e-9) { continue; }
      return norm > 1e-9 ? (dx*gx + dy*gy) / (length*norm) : 1.;
    }
    return 1.;
  };
  // A short start connector can lead to an expensive suffix with many cusps.
  // Compare feasible complete routes, bounded by the existing candidate/time caps.
  std::vector<std::pair<double, nav_msgs::msg::Path>> feasible;
  if (ordinary && enough_time()) {
    try {
      auto path=native_plan(start);
      std::vector<Pose> poses;
      for (const auto & p:path.poses) { poses.push_back(poseOf(p)); }
      const double cost=maneuverCost(poses);
      if (std::isfinite(cost)) { feasible.emplace_back(cost,std::move(path)); }
    } catch (const std::runtime_error &) {
      // An independently checked exact connection can survive native failure.
    }
  }
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
      const double base_cost = maneuverCost(combined);
      const double first_cos = first_cosine(result);
      const double penalty = (reverse_start_penalty_ > 0. && first_cos < -0.5)
        ? reverse_start_penalty_ : 0.;
      const double cost = base_cost + penalty;
      if (std::isfinite(cost)) {
        RCLCPP_INFO(_logger,
          "connector_candidate: cost=%.3f base=%.3f penalty=%.3f poses=%zu connector_poses=%zu cusps=%zu first_cos=%.3f",
          cost, base_cost, penalty, result.poses.size(), c.poses.size(),
          cusp_count(result), first_cos);
        feasible.emplace_back(cost, std::move(result));
      }
    }
  }
  std::stable_sort(feasible.begin(), feasible.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });
  // Compare current-map safety and integrated inflation risk in one locked
  // map version. A shorter exact connection must not bypass a safer detour.
  // Unknown is the maximum traversable risk, never a free-space reward.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
  if (_costmap->getResolution()!=resolution_) {
    throw nav2_core::PlannerException("Costmap geometry changed during route selection");
  }
  const auto current_body=_costmap_ros->getRobotFootprint();
  auto checked_risk = [&](const nav_msgs::msg::Path & path) {
    const double invalid=std::numeric_limits<double>::infinity();
    if (path.poses.size()<2) { return invalid; }
    for (const auto & p:path.poses) {
      if (Clock::now()>=deadline || !safe(*_costmap,current_body,poseOf(p),_allow_unknown)) {
        return invalid;
      }
    }
    double risk=0.;
    for (size_t i=1;i<path.poses.size();++i) {
      const auto a=poseOf(path.poses[i-1]),b=poseOf(path.poses[i]);
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      const auto samples=static_cast<unsigned>(std::ceil(length/std::min(.05,resolution_/4)));
      for (unsigned j=0;j<samples;++j) {
        if (Clock::now()>=deadline) { return invalid; }
        unsigned x,y; const double t=(j+.5)/samples;
        if (!_costmap->worldToMap(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,x,y)) { return invalid; }
        const auto cost=_costmap->getCost(x,y);
        if (cost==253 || cost==254 || (cost==255 && !_allow_unknown)) { return invalid; }
        risk+=length/samples*(cost==255?1.:cost/252.);
      }
    }
    return risk;
  };
  const double direct_risk=checked_risk(direct);
  const double direct_first_cos=first_cosine(direct);
  const double direct_penalty=(reverse_start_penalty_>0. && direct_first_cos<-0.5)
    ? reverse_start_penalty_ : 0.;
  const double direct_cost=maneuverCost(goal_curve.poses)+direct_penalty;
  for (auto & candidate : feasible) {
    const double native_risk=checked_risk(candidate.second);
    if (std::isfinite(native_risk)) {
      const auto end=poseOf(candidate.second.poses.back());
      // Native tolerance can stop short of the requested endpoint. Its
      // remaining Euclidean gap is a lower bound on finishing the same task.
      const double completion_bound=candidate.first+std::hypot(end.x-goal.pose.position.x,
        end.y-goal.pose.position.y);
      if (std::isfinite(direct_risk) && direct_cost<=completion_bound+1e-6 &&
        direct_risk<=native_risk+1e-9) {
        if (Clock::now()>=deadline) { break; }
        RCLCPP_INFO(_logger,"continuous_goal_selected: cost=%.6f risk=%.6f cusps=%zu first_cos=%.3f",
          direct_cost,direct_risk,cusp_count(direct),first_cosine(direct));
        return direct;
      }
      if (Clock::now()>=deadline) { break; }
      RCLCPP_INFO(_logger, "continuous_complete_selected: cost=%.6f candidates=%zu cusps=%zu first_cos=%.3f",
        candidate.first, feasible.size(), cusp_count(candidate.second), first_cosine(candidate.second));
      return candidate.second;
    }
  }
  if (std::isfinite(direct_risk) && Clock::now()<deadline) {
    RCLCPP_INFO(_logger,"continuous_goal_selected: no safe lattice route, cost=%.6f cusps=%zu first_cos=%.3f",
      direct_cost,cusp_count(direct),first_cosine(direct));
    return direct;
  }
  throw nav2_core::PlannerException("No safe bounded continuous-start plan");
}
}  // namespace wildos_ackermann_smac
PLUGINLIB_EXPORT_CLASS(wildos_ackermann_smac::Planner, nav2_core::GlobalPlanner)
