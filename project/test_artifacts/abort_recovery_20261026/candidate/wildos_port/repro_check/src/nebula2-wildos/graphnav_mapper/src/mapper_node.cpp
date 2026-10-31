#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

#include "graphnav_mapper/mapper.hpp"

namespace graphnav_mapper
{

bool occupancy_map_to_grid_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_map,
                               grid_map_msgs::msg::GridMap::SharedPtr grid_map)
{
  const auto& info = occ_map->info;
  const auto& p = info.origin.position;
  const auto& q = info.origin.orientation;
  const size_t cells = static_cast<size_t>(info.width) * info.height;
  if (occ_map->header.frame_id.empty() || info.width == 0 || info.height == 0 ||
      cells > std::numeric_limits<uint32_t>::max() || cells != occ_map->data.size() ||
      !std::isfinite(info.resolution) || info.resolution <= 0 ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 ||
      std::abs(std::abs(q.w) - 1.0) > 1e-6 ||
      std::any_of(occ_map->data.begin(), occ_map->data.end(), [](int v) { return v < -1 || v > 100; }))
    return false;
  grid_map->header = occ_map->header;
  grid_map->info.resolution = occ_map->info.resolution;
  size_t size_x = occ_map->info.width;
  size_t size_y = occ_map->info.height;
  grid_map->info.length_x = size_x * occ_map->info.resolution;
  grid_map->info.length_y = size_y * occ_map->info.resolution;
  grid_map->info.pose.position.x = occ_map->info.origin.position.x + (size_x)*occ_map->info.resolution / 2.0;
  grid_map->info.pose.position.y = occ_map->info.origin.position.y + (size_y)*occ_map->info.resolution / 2.0;
  grid_map->info.pose.position.z = occ_map->info.origin.position.z;
  grid_map->info.pose.orientation.x = 0;
  grid_map->info.pose.orientation.y = 0;
  grid_map->info.pose.orientation.z = 0;
  grid_map->info.pose.orientation.w = 1;
  grid_map->layers = { "traversability", "unknown", "elevation", "elevation_valid" };
  grid_map->data.resize(4);
  grid_map->data[0].layout.data_offset = 0;
  grid_map->data[0].layout.dim.resize(2);
  grid_map->data[0].layout.dim[0].label = "column_index";
  grid_map->data[0].layout.dim[0].size = size_y;
  grid_map->data[0].layout.dim[0].stride = size_y * size_x;
  grid_map->data[0].layout.dim[1].label = "row_index";
  grid_map->data[0].layout.dim[1].size = size_x;
  grid_map->data[0].layout.dim[1].stride = size_x;
  grid_map->data[1].layout = grid_map->data[0].layout;
  grid_map->data[2].layout = grid_map->data[0].layout;
  grid_map->data[3].layout = grid_map->data[0].layout;
  grid_map->data[0].data.resize(size_x * size_y);
  grid_map->data[1].data.resize(size_x * size_y);
  grid_map->data[2].data.resize(size_x * size_y);
  grid_map->data[3].data.assign(size_x * size_y, 0.0f);
  for (size_t x = 0; x < size_x; x++)
  {
    for (size_t y = 0; y < size_y; y++)
    {
      size_t occ_map_index = y * size_x + x;
      size_t grid_map_index = (size_y - 1 - y) * size_x + (size_x - 1 - x);
      grid_map->data[0].data[grid_map_index] = std::clamp<int>(occ_map->data[occ_map_index], 0, 100) / 100.0;
      grid_map->data[1].data[grid_map_index] = occ_map->data[occ_map_index] == -1 ? 1.0 : 0.0;
      grid_map->data[2].data[grid_map_index] = 0.0;
    }
  }
  return true;
}

class MapperNode : public rclcpp::Node
{
public:
  MapperNode(const rclcpp::NodeOptions& options)
    : Node("mapper_node", options)
    , tf_buffer_(this->get_clock())
    // Dynamic poses are repeated sensor data. Reliable delivery can hold new
    // poses behind a delayed sample; exact-time map lookup still rejects gaps.
    // Leave static TF on its default reliable/transient-local QoS.
    , tf_listener_(tf_buffer_, this, true, tf2_ros::DynamicListenerQoS().best_effort())
    , mapper_(tf_buffer_, this->get_logger())
  {
    rcl_interfaces::msg::ParameterDescriptor frozen_profile;
    frozen_profile.read_only = true;
    frozen_profile.description = "Applied at startup; restart to change the robot profile";
    mapper_.robot_profile_id_ = this->declare_parameter<std::string>("robot_profile_id", "default", frozen_profile);
    mapper_.physical_max_slope_ = this->declare_parameter<double>("robot_max_slope", 0.35, frozen_profile);
    mapper_.physical_max_step_height_ = this->declare_parameter<double>("robot_max_step_height", 0.10, frozen_profile);
    mapper_.physical_max_roughness_ = this->declare_parameter<double>("robot_max_roughness", 0.08, frozen_profile);
    mapper_.robot_minimum_clearance_ = this->declare_parameter<double>("robot_minimum_clearance", 0.30, frozen_profile);
    mapper_.robot_body_front_extent_ = this->declare_parameter<double>("robot_body_front_extent", -1., frozen_profile);
    mapper_.robot_body_rear_extent_ = this->declare_parameter<double>("robot_body_rear_extent", -1., frozen_profile);
    mapper_.robot_body_half_width_ = this->declare_parameter<double>("robot_body_half_width", -1., frozen_profile);
    mapper_.robot_body_height_ = this->declare_parameter<double>("robot_body_height", -1., frozen_profile);
    mapper_.robot_footprint_padding_ = this->declare_parameter<double>("robot_footprint_padding", 0., frozen_profile);
    mapper_.robot_ground_clearance_ = this->declare_parameter<double>("robot_ground_clearance", -1., frozen_profile);
    mapper_.robot_min_turning_radius_ = this->declare_parameter<double>("robot_min_turning_radius", -1., frozen_profile);
    mapper_.robot_holonomic_ = this->declare_parameter<bool>("robot_holonomic", true, frozen_profile);
    mapper_.validate_robot_profile();
    dbg_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("dbg_map", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(1), [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          const auto& stamp = msg->header.stamp;
          const auto& p = msg->pose.pose.position;
          const auto& q = msg->pose.pose.orientation;
          if (stamp.sec < 0 || stamp.nanosec >= 1000000000u || msg->header.frame_id.empty() ||
              !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
              !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
              std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > .1 ||
              (odometry_received_ && (stamp.sec < latest_sensor_stamp_.sec ||
                  (stamp.sec == latest_sensor_stamp_.sec && stamp.nanosec <= latest_sensor_stamp_.nanosec))))
            return;
          if (pose_graph_received_)
          {
            this->mapper_.odometry_update(msg);
            latest_sensor_stamp_ = msg->header.stamp;
            odometry_received_ = true;
          }
        });
    traversability_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "traversability", rclcpp::QoS(1), std::bind(&MapperNode::traversability_cb, this, std::placeholders::_1));
    if (this->declare_parameter<bool>("use_occupancy_input", true, frozen_profile))
      occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "occupancy", rclcpp::QoS(1), [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          auto grid_map = std::make_shared<grid_map_msgs::msg::GridMap>();
          if (occupancy_map_to_grid_map(msg, grid_map))
            traversability_cb(grid_map);
          else
            RCLCPP_WARN(this->get_logger(), "Ignoring malformed or rotated occupancy grid");
        });
    pose_graph_sub_ = this->create_subscription<pose_graph_msgs::msg::PoseGraph>(
        "pose_graph", 10, [this](const pose_graph_msgs::msg::PoseGraph::ConstSharedPtr msg) {
          try
          {
            this->mapper_.pose_graph_update(msg);
            pose_graph_received_ = true;
          }
          catch (const std::invalid_argument& e)
          {
            RCLCPP_ERROR(this->get_logger(), "Error updating pose graph: %s", e.what());
          }
        });
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("nav_graph_vis", 10);
    graph_pub_ = this->create_publisher<graphnav_msgs::msg::NavigationGraph>("nav_graph", 1);
    perception_pub_ = this->create_publisher<graphnav_msgs::msg::NavigationGraph>("nav_graph_perception", 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (pose_graph_received_ && odometry_received_)
      {
        auto markers = this->mapper_.get_visualization(this->get_clock()->now(), true);
        this->marker_pub_->publish(markers);
        // The graph participates in camera/odom approximate synchronization.
        // Publication wall time can differ from sensor time after pauses and
        // the bridge's configured timestamp offset.
        auto graph = this->mapper_.get_graph_msg(rclcpp::Time(latest_sensor_stamp_));
        this->graph_pub_->publish(graph);
        graph.edges.clear();
        this->perception_pub_->publish(graph);
      }
    });
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr traversability_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<pose_graph_msgs::msg::PoseGraph>::SharedPtr pose_graph_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr dbg_map_pub_;
  rclcpp::Publisher<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_pub_;
  rclcpp::Publisher<graphnav_msgs::msg::NavigationGraph>::SharedPtr perception_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Mapper mapper_;
  bool pose_graph_received_ = false;
  bool odometry_received_ = false;
  builtin_interfaces::msg::Time latest_sensor_stamp_;

  void traversability_cb(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    if (pose_graph_received_)
    {
      std::shared_ptr<grid_map_msgs::msg::GridMap> dbg_map = nullptr;
      if (dbg_map_pub_->get_subscription_count() > 0)
      {
        dbg_map = std::make_shared<grid_map_msgs::msg::GridMap>();
      }
      this->mapper_.traversability_update(msg, dbg_map);
      if (dbg_map)
      {
        dbg_map_pub_->publish(*dbg_map);
      }
    }
  }
};

}  // namespace graphnav_mapper

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_mapper::MapperNode)
