// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_map>
#include "wildos_ackermann_rpp/path_progress.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace wildos_ackermann_rpp {
std::shared_ptr<PathProgress> path_progress(const std::string & key) {
  static std::mutex mutex;
  static std::unordered_map<std::string,std::weak_ptr<PathProgress>> registry;
  std::lock_guard<std::mutex> lock(mutex);
  auto & entry=registry[key];
  auto state=entry.lock();
  if (!state) {state=std::make_shared<PathProgress>();entry=state;}
  return state;
}
// Keep this library independent of Nav2's goal-checker plugin library. An OS
// dependency would register its factories under the wrong class-loader owner.
class ContinuousGoalChecker : public nav2_core::GoalChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
    auto node=parent.lock();
    std::vector<rclcpp::Parameter> frozen;
    auto parameter=[&](const std::string & field,const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node,name+"."+field,value);
      auto p=node->get_parameter(name+"."+field);frozen.push_back(p);return p;
    };
    xy_=parameter("xy_goal_tolerance",rclcpp::ParameterValue(.25)).as_double();
    yaw_=parameter("yaw_goal_tolerance",rclcpp::ParameterValue(.25)).as_double();
    stateful_=parameter("stateful",rclcpp::ParameterValue(true)).as_bool();
    auto controller=parameter("controller_id",rclcpp::ParameterValue(std::string("FollowPath"))).as_string();
    if (stopped_) {
      trans_=parameter("trans_stopped_velocity",rclcpp::ParameterValue(.05)).as_double();
      rot_=parameter("rot_stopped_velocity",rclcpp::ParameterValue(.05)).as_double();
    }
    if (!std::isfinite(xy_) || !std::isfinite(yaw_) || xy_<=0. || yaw_<=0. ||
        !std::isfinite(trans_) || !std::isfinite(rot_) || trans_<0. || rot_<0.) {
      throw std::invalid_argument("Invalid continuous goal-checker tolerances");
    }
    progress_=path_progress(std::string(node->get_node_base_interface()->get_fully_qualified_name())+"/"+controller);
    guard_=node->add_on_set_parameters_callback([frozen](const auto & parameters) {
      rcl_interfaces::msg::SetParametersResult result;result.successful=true;
      for (const auto & p:parameters) {for (const auto & expected:frozen) {
        if (p.get_name()==expected.get_name() && p.get_parameter_value()!=expected.get_parameter_value()) {
          result.successful=false;result.reason="Path completion changes require controller reconfigure";
        }
      }}
      return result;
    });
    reset();
  }
  void reset() override {check_xy_=true;}
  bool isGoalReached(const geometry_msgs::msg::Pose & actual,
    const geometry_msgs::msg::Pose & goal,const geometry_msgs::msg::Twist & speed) override {
    if (!progress_->active.load() || !progress_->finish_allowed.load()) {return false;}
    if (check_xy_) {
      if (std::hypot(actual.position.x-goal.position.x,actual.position.y-goal.position.y)>xy_) {return false;}
      if (stateful_) {check_xy_=false;}
    }
    const auto heading=[](const auto & q) {
      return std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
    };
    if (std::abs(std::remainder(heading(actual.orientation)-heading(goal.orientation),2.*std::acos(-1.)))>yaw_) {return false;}
    return !stopped_ || (std::hypot(speed.linear.x,speed.linear.y)<=trans_ && std::abs(speed.angular.z)<=rot_);
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose,geometry_msgs::msg::Twist & velocity) override {
    pose.position.x=pose.position.y=xy_;pose.position.z=std::numeric_limits<double>::lowest();
    pose.orientation.z=std::sin(yaw_/2.);pose.orientation.w=std::cos(yaw_/2.);
    velocity.linear.x=velocity.linear.y=stopped_?trans_:std::numeric_limits<double>::lowest();
    velocity.angular.z=stopped_?rot_:std::numeric_limits<double>::lowest();
    return true;
  }
protected:
  bool stopped_=false;
private:
  double xy_=.25,yaw_=.25,trans_=.05,rot_=.05;
  bool stateful_=true,check_xy_=true;
  std::shared_ptr<PathProgress> progress_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
};
class ContinuousStoppedGoalChecker : public ContinuousGoalChecker {
public:ContinuousStoppedGoalChecker() {stopped_=true;}
};
}
PLUGINLIB_EXPORT_CLASS(wildos_ackermann_rpp::ContinuousGoalChecker,nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(wildos_ackermann_rpp::ContinuousStoppedGoalChecker,nav2_core::GoalChecker)
