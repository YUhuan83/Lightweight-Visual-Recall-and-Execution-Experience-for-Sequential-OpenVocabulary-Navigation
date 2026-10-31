#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <cstring>
#include <nlohmann/json.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using json = nlohmann::json;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  if (argc < 2) return 2;
  json fixture; std::ifstream(argv[1]) >> fixture;
  auto node=std::make_shared<nav2_util::LifecycleNode>("observation_buffer_probe");
  auto clock=node->get_clock();
  if(rcl_enable_ros_time_override(clock->get_clock_handle())!=RCL_RET_OK) return 3;
  tf2_ros::Buffer tf(clock);
  json report=json::array();
  for(double keep:{0.,1.}) {
    nav2_costmap_2d::ObservationBuffer buffer(node,"/review",keep,0.,-1000.,1000.,
        1000.,0.,1000.,0.,tf,"spot1/odom","spot1/odom",tf2::durationFromSec(0.));
    for(const auto & frame:fixture["frames"]) {
      rcl_set_ros_time_override(clock->get_clock_handle(),frame["receipt_ns"].get<int64_t>());
      std::ifstream input(frame["file"].get<std::string>(),std::ios::binary);
      std::vector<char> raw((std::istreambuf_iterator<char>(input)),{});
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id="spot1/odom";
      cloud.header.stamp=rclcpp::Time(frame["source_stamp_ns"].get<int64_t>());
      cloud.height=1; cloud.width=raw.size()/12; cloud.is_dense=true;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,
          "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(raw.size()/12);
      std::memcpy(cloud.data.data(),raw.data(),raw.size());
      buffer.bufferCloud(cloud);
    }
    rcl_set_ros_time_override(clock->get_clock_handle(),fixture["map_stamp_ns"].get<int64_t>());
    auto evaluate=[&]() {
      std::vector<nav2_costmap_2d::Observation> observations;
      buffer.getObservations(observations);
      nav2_costmap_2d::Costmap2D map(fixture["width"],fixture["height"],fixture["resolution"],
          fixture["origin"][0],fixture["origin"][1],0);
      std::vector<int64_t> stamps;
      for(const auto & observation:observations) {
        const auto & cloud=*observation.cloud_;
        stamps.push_back(rclcpp::Time(cloud.header.stamp).nanoseconds());
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y");
        for(;x!=x.end();++x,++y) {
          unsigned mx,my;
          if(map.worldToMap(*x,*y,mx,my))map.setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
      nav2_costmap_2d::Footprint footprint;
      for(auto p:std::vector<std::pair<double,double>>{{3.41,1.06},{3.41,-1.06},{-1.06,-1.06},{-1.06,1.06}}) {
        geometry_msgs::msg::Point point;point.x=p.first;point.y=p.second;footprint.push_back(point);
      }
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
      double cost=checker.footprintCostAtPose(fixture["pose"][0],fixture["pose"][1],fixture["pose"][2],footprint);
      return json{{"observations",observations.size()},{"source_stamps",stamps},{"native_footprint_cost",cost}};
    };
    auto current=evaluate();
    rcl_set_ros_time_override(clock->get_clock_handle(),fixture["map_stamp_ns"].get<int64_t>()+1'100'000'000);
    auto expired=evaluate();
    report.push_back({{"keep_s",keep},{"at_update",current},{"after_1_1s_without_input",expired}});
  }
  std::cout << report.dump(2) << std::endl;
  rclcpp::shutdown();
  const bool reproduced=report[0]["at_update"]["native_footprint_cost"].get<double>()<254. &&
      report[1]["at_update"]["native_footprint_cost"].get<double>()==254.;
  return reproduced ? 0 : 1;
}
