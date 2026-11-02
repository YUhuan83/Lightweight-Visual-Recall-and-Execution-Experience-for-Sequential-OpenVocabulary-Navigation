#include <gtest/gtest.h>
#include "wildos_ackermann_smac/connector.hpp"
TEST(SmacGeometry, ContinuousConnectionRespectsRadius) {
  EXPECT_TRUE(wildos_ackermann_smac::connect({0.,0.,0.},{12.,3.,.3},12.,.4,.1).poses.empty());
  const auto curve=wildos_ackermann_smac::connect({0.,0.,0.},{12.,3.,.3},12.,.4,.1,40.);
  ASSERT_GT(curve.poses.size(),2u);
  EXPECT_NEAR(curve.poses.back().x,12.,1e-5);
  EXPECT_NEAR(curve.poses.back().y,3.,1e-5);
  for(size_t i=1;i<curve.poses.size();++i) {
    const auto & a=curve.poses[i-1];const auto & b=curve.poses[i];
    const double d=std::hypot(a.x-b.x,a.y-b.y);
    if(d>1e-6)EXPECT_LE(std::abs(std::remainder(b.yaw-a.yaw,2*std::acos(-1.)))/d,1./12.+.001);
  }
}
TEST(SmacGeometry, RejectsOccupiedVehicleCenter) {
  nav2_costmap_2d::Costmap2D map(100,100,.5,-25.,-25.,0);
  nav2_costmap_2d::Footprint footprint;
  for(const auto & xy:std::vector<std::pair<double,double>>{{3.4,1.05},{3.4,-1.05},{-1.05,-1.05},{-1.05,1.05}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  EXPECT_TRUE(wildos_ackermann_smac::safe(map,footprint,{0.,0.,0.},true));
  map.setCost(50,50,254);
  EXPECT_FALSE(wildos_ackermann_smac::safe(map,footprint,{0.,0.,0.},true));
}
