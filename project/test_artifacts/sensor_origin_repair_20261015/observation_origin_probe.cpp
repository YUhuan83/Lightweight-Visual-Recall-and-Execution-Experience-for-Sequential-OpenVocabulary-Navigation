#include <cassert>
#include <cmath>
#include <iostream>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
int main(int argc,char** argv){
 rclcpp::init(argc,argv);auto node=std::make_shared<nav2_util::LifecycleNode>("ray_origin_probe");
 tf2_ros::Buffer tf(node->get_clock());tf.setUsingDedicatedThread(true);auto now=node->now();
 geometry_msgs::msg::TransformStamped a;a.header.stamp=now;a.header.frame_id="odom";a.child_frame_id="base";
 a.transform.translation.x=100.;a.transform.translation.y=200.;a.transform.translation.z=50.;a.transform.rotation.w=1.;
 tf.setTransform(a,"fixture",true);
 a.header.frame_id="base";a.child_frame_id="aperture";a.transform.translation.x=1.252;a.transform.translation.y=0.;a.transform.translation.z=1.56;
 tf.setTransform(a,"fixture",true);
 sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id="base";cloud.header.stamp=now;
 sensor_msgs::PointCloud2Modifier mod(cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(1);
 sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");*x=2.;*y=2.;*z=-.15;
 std::vector<nav2_costmap_2d::Observation> observations[2];
 for(int i=0;i<2;i++){
  nav2_costmap_2d::ObservationBuffer buffer(node,"points",0.,0.,-1000.,1000.,25.,0.,30.,0.,tf,"odom",i?"aperture":"",tf2::durationFromSec(0.));
  buffer.bufferCloud(cloud);buffer.getObservations(observations[i]);assert(observations[i].size()==1);
  auto& o=observations[i][0];sensor_msgs::PointCloud2ConstIterator<float> px(*o.cloud_,"x"),py(*o.cloud_,"y"),pz(*o.cloud_,"z");
  assert(std::abs(*px-102)<1e-5 && std::abs(*py-202)<1e-5 && std::abs(*pz-49.85)<1e-5);
  assert(std::abs(o.origin_.x-(100.+(i?1.252:0)))<1e-8);
  assert(std::abs(o.origin_.z-(50.+(i?1.56:0)))<1e-8);
  std::cout<<"explicit_sensor_frame="<<i<<" origin="<<o.origin_.x<<","<<o.origin_.y<<","<<o.origin_.z<<" endpoint="<<*px<<","<<*py<<","<<*pz<<"\n";
 }
 // The old rear-axle ray passes through (101,201); the true aperture ray
 // reaches x=101.626 at y=201. These are different observed-free cells.
 auto& old=observations[0][0].origin_;auto& actual=observations[1][0].origin_;
 double old_at_y=old.x+(102.-old.x)*(201.-old.y)/(202.-old.y);
 double true_at_y=actual.x+(102.-actual.x)*(201.-actual.y)/(202.-actual.y);
 assert(std::abs(old_at_y-101.)<1e-9);assert(true_at_y>101.5);
 std::cout<<"PASS unchanged endpoints, corrected ray origin; false clear cell counterexample\n";
 node.reset();rclcpp::shutdown();
}
