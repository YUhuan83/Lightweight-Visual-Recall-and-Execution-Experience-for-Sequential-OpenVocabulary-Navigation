#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "graphnav_planner/planner.hpp"
#include "graphnav_planner/graph_score_overlay.hpp"
#include "graphnav_planner/exploration_utility.hpp"
#include "graphnav_planner/exploration_backoff.hpp"

namespace graphnav_planner {
class PlannerNode : public rclcpp::Node {
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options)
  : Node("planner_node", options), buffer_(get_clock()), listener_(buffer_, this), planner_(get_logger()) {
    planner_.frontier_dist_cost_factor_=declare_parameter("frontier_dist_cost_factor",2.0);
    planner_.goal_dist_cost_factor_=declare_parameter("goal_dist_cost_factor",1.0);
    planner_.frontier_score_factor_=declare_parameter("frontier_score_factor",20.0);
    planner_.min_local_frontier_score_=declare_parameter("min_local_frontier_score",.4);
    planner_.local_frontier_radius_=declare_parameter("local_frontier_radius",10.0);
    planner_.path_smoothness_period_=0.;
    planner_.set_trav_class("default");
    topology_sub_=create_subscription<graphnav_msgs::msg::NavigationGraph>(
      "/spot1/nav_graph",1,[this](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg){overlay_.remember(msg);});
    score_sub_=create_subscription<graphnav_msgs::msg::NavigationGraph>(
      "/spot1/scored_nav_graph",1,[this](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg){
        auto graph=overlay_.combine(*msg);
        if(!graph || graph->nodes.empty() || graph->current_node_idx>=graph->nodes.size())return;
        graph_=graph;planner_.update_graph(graph_);plan();
      });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(
      "/spot1/odom",10,[this](const nav_msgs::msg::Odometry::ConstSharedPtr msg){odom_=msg;});
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(
      "/spot1/graphnav_destination",10,[this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        if(msg->header.frame_id!="spot1/odom")return;
        goal_=*msg;plan();
      });
    clear_=create_service<std_srvs::srv::Trigger>("~/clear_goal",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response){
        goal_.reset();selected_.reset();backoff_.clear();response->success=true;
      });
    unreachable_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/unreachable",10,[this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        if(goal_ || !odom_)return;
        const auto stamp=now().nanoseconds();
        const auto identity=backoff_.identity(*msg,stamp);
        if(!identity || !backoff_.reject(*msg,stamp,true,odom_->pose.pose.position))return;
        if(selected_ && selected_uuid_==*identity)selected_.reset();
        plan();
      });
    path_pub_=create_publisher<nav_msgs::msg::Path>("~/path",1);
    timer_=create_wall_timer(std::chrono::milliseconds(1000),[this]{plan();});
  }

private:
  std::optional<geometry_msgs::msg::PoseStamped> exploration_goal() {
    const auto trav=std::find(graph_->trav_classes.begin(),graph_->trav_classes.end(),"default");
    if(trav==graph_->trav_classes.end())return std::nullopt;
    const size_t index=std::distance(graph_->trav_classes.begin(),trav);
    const auto & origin=odom_->pose.pose.position;
    const auto selection_time=now().nanoseconds();
    if(selected_ && backoff_.postponed(selected_->pose.position,selection_time,origin))selected_.reset();
    const auto frontier_score = [](const graphnav_msgs::msg::Node & node,
                                   const geometry_msgs::msg::Point & endpoint) {
      const std::vector<float>* scores=nullptr;
      for(const auto & property:node.properties)if(property.key=="frontier_scores")scores=&property.value;
      double score=.3;
      if(scores && !scores->empty()) {
        const double bearing=std::atan2(endpoint.y-node.pose.position.y,endpoint.x-node.pose.position.x);
        const double turn=std::acos(-1.)*2.;
        const size_t bin=static_cast<size_t>(std::lround((bearing<0?bearing+turn:bearing)/turn*scores->size()))%scores->size();
        score=(*scores)[bin];
      }
      return score;
    };
    const auto frontier_utility = [&](const graphnav_msgs::msg::Node & node,
                                      const geometry_msgs::msg::Point & endpoint,
                                      const std::string & id) {
      const double range=std::hypot(endpoint.x-origin.x,endpoint.y-origin.y);
      if(!std::isfinite(range) || range<3.)return -std::numeric_limits<double>::infinity();
      return frontier_score(node,endpoint)-.004*range-frontier_visit_penalty*visits_[id];
    };

    bool selected_is_frontier=false;
    bool selected_reached=false;
    double selected_utility=-std::numeric_limits<double>::infinity();
    if(selected_) {
      selected_reached=std::hypot(selected_->pose.position.x-origin.x,selected_->pose.position.y-origin.y)<3.;
      for(const auto & node:graph_->nodes) {
        if(uuid_to_string(node.uuid)!=selected_uuid_ || index>=node.trav_properties.size() ||
           !node.trav_properties[index].is_frontier)continue;
        if(!frontier_endpoint_current(selected_->pose.position,
            node.trav_properties[index].frontier_points))continue;
        selected_is_frontier=true;
        selected_utility=frontier_utility(node,selected_->pose.position,selected_uuid_);
        break;
      }
      if(selected_reached)++visits_[selected_uuid_];
      if(selected_reached || !selected_is_frontier)selected_.reset();
    }

    double best=-std::numeric_limits<double>::infinity();
    std::optional<geometry_msgs::msg::PoseStamped> replacement;
    std::string replacement_uuid;
    for(const auto & n:graph_->nodes) {
      if(index>=n.trav_properties.size() || !n.trav_properties[index].is_frontier)continue;
      const auto id=uuid_to_string(n.uuid);
      for(const auto & endpoint:n.trav_properties[index].frontier_points) {
        if(backoff_.postponed(endpoint,selection_time,origin))continue;
        const double score=frontier_score(n,endpoint);
        if(!std::isfinite(score) || score<=1e-4)continue;
        const double utility=frontier_utility(n,endpoint,id);
        // Keep the accepted route through small score fluctuations.
        if(selected_ && selected_is_frontier &&
           !frontier_candidate_should_preempt(utility,selected_utility))continue;
        if(!std::isfinite(utility) || utility<=best)continue;
        geometry_msgs::msg::PoseStamped candidate;
        candidate.header=graph_->header;
        candidate.pose.position=endpoint;candidate.pose.orientation.w=1.;
        Planner probe=planner_;
        if(probe.plan_to_frontier(id,endpoint,now()).empty())continue;
        best=utility;replacement=candidate;replacement_uuid=id;
      }
    }
    if(replacement) {selected_=replacement;selected_uuid_=replacement_uuid;}
    return selected_;
  }

  void plan() {
    if(!graph_ || !odom_)return;
    if((now()-rclcpp::Time(graph_->header.stamp)).seconds()>5.)return;
    auto destination=goal_ ? goal_ : exploration_goal();
    if(!destination)return;
    auto pose=*destination;
    pose.header.stamp=graph_->header.stamp;
    geometry_msgs::msg::PoseStamped transformed;
    try {transformed=buffer_.transform(pose,graph_->header.frame_id,tf2::durationFromSec(.1));}
    catch(const tf2::TransformException &){return;}
    const auto & p=transformed.pose.position;
    Eigen::Vector3d target(p.x,p.y,p.z);
    auto path=goal_ ? planner_.plan_to_goal(target,3.,now(),graph_) :
        planner_.plan_to_frontier(selected_uuid_,p,now());
    if(path.empty())return;
    if(goal_ && !planner_.route_complete() && (path.back()-target).norm()>1e-6)path.push_back(target);
    nav_msgs::msg::Path output;output.header=graph_->header;
    for(size_t i=0;i<path.size();++i) {
      geometry_msgs::msg::PoseStamped waypoint;waypoint.header=output.header;
      waypoint.pose.position.x=path[i].x();waypoint.pose.position.y=path[i].y();waypoint.pose.position.z=path[i].z();
      const auto & previous=path[i?i-1:0];
      const double angle=i?std::atan2(path[i].y()-previous.y(),path[i].x()-previous.x()):0.;
      waypoint.pose.orientation.z=std::sin(angle/2.);waypoint.pose.orientation.w=std::cos(angle/2.);
      output.poses.push_back(waypoint);
    }
    if(!goal_ && selected_ && !output.poses.empty())
      backoff_.remember(output.header,output.poses.back().pose.position,
        selected_->pose.position,selected_uuid_);
    path_pub_->publish(output);
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  Planner planner_;
  GraphScoreOverlay overlay_;
  ExplorationBackoff backoff_;
  graphnav_msgs::msg::NavigationGraph::ConstSharedPtr graph_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_,selected_;
  std::string selected_uuid_;
  std::unordered_map<std::string,int> visits_;
  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr topology_sub_,score_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr unreachable_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
