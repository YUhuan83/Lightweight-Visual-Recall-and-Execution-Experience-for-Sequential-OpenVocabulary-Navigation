// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "wildos_ackermann_smac/connector.hpp"
using namespace wildos_ackermann_smac;
static nav2_costmap_2d::Footprint body() {
  nav2_costmap_2d::Footprint f;
  for (auto xy : std::vector<std::pair<double,double>>{{3.41,1.06},{3.41,-1.06},{-1.06,-1.06},{-1.06,1.06}}) {
    geometry_msgs::msg::Point p; p.x=xy.first; p.y=xy.second; f.push_back(p);
  }
  return f;
}
TEST(Connection, ExactForwardReverseAndArc) {
  for (double d : {-1.,1.}) {
    auto c=connect({0,0,0},{2*d,0,0},8,.4,.05);
    ASSERT_GT(c.poses.size(),2u); EXPECT_NEAR(c.length,2.,1e-9); EXPECT_EQ(c.cusps,0u);
    EXPECT_NEAR(c.poses.back().x,2*d,1e-9);
  }
  auto c=connect({0,0,0},{8*std::sin(.25),8*(1-std::cos(.25)),.25},8,.4,.05);
  ASSERT_GT(c.poses.size(),2u); EXPECT_NEAR(c.length,2.,1e-8);
  for (unsigned i=1;i<c.poses.size();++i) {
    double distance=std::hypot(c.poses[i].x-c.poses[i-1].x,c.poses[i].y-c.poses[i-1].y);
    EXPECT_LE(distance,.05+1e-9);
    EXPECT_NEAR(std::abs(c.poses[i].yaw-c.poses[i-1].yaw)/distance,1./8,1e-6);
  }
}
TEST(Connection, RejectsShortNonfiniteAndAllocationOverflow) {
  EXPECT_TRUE(connect({0,0,0},{.2,0,0},8,.4,.05).poses.empty());
  EXPECT_TRUE(connect({0,0,0},{0,0,M_PI},8,.4,.05).poses.empty());
  EXPECT_TRUE(connect({0,0,0},{2,0,0},0,.4,.05).poses.empty());
  EXPECT_TRUE(connect({NAN,0,0},{2,0,0},8,.4,.05).poses.empty());
  EXPECT_TRUE(connect({0,0,0},{2,0,0},8,.4,1e-10).poses.empty());
}
TEST(Collision, InteriorBoundaryUnknownAndRecovery) {
  nav2_costmap_2d::Costmap2D map(320,320,.25,-40,-40,0); auto f=body();
  ASSERT_TRUE(safe(map,f,{0,0,0},true));
  unsigned x,y; map.worldToMap(2,0,x,y); map.setCost(x,y,254);
  EXPECT_FALSE(safe(map,f,{0,0,0},true)); map.setCost(x,y,255);
  EXPECT_TRUE(safe(map,f,{0,0,0},true)); EXPECT_FALSE(safe(map,f,{0,0,0},false));
  map.setCost(x,y,0); map.worldToMap(3.5,0,x,y); map.setCost(x,y,254);
  EXPECT_FALSE(safe(map,f,{.125,.125,0},true)); map.setCost(x,y,0);
  EXPECT_TRUE(safe(map,f,{.125,.125,0},true));
  EXPECT_FALSE(safe(map,f,{100,0,0},true));
  EXPECT_FALSE(safe(map,{}, {0,0,0},true));
}
TEST(Search, DeadlineAndEvaluationBounds) {
  nav2_costmap_2d::Costmap2D map(320,320,.25,-40,-40,0); Limits limits;
  auto expired=search(map,body(),{0,0,0},limits,Clock::now());
  EXPECT_TRUE(expired.timed_out); EXPECT_EQ(expired.evaluated,0u); EXPECT_TRUE(expired.curves.empty());
  limits.max_evaluations=5;
  auto limited=search(map,body(),{0,0,0},limits,Clock::now()+std::chrono::seconds(1));
  EXPECT_LE(limited.evaluated,5u);
  limits.max_evaluations=6000;
  auto valid=search(map,body(),{.02,.02,.036},limits,Clock::now()+std::chrono::seconds(1));
  ASSERT_FALSE(valid.curves.empty()); EXPECT_LE(valid.curves.size(),6u);
  for(const auto & c:valid.curves) {
    ASSERT_FALSE(c.poses.empty()); EXPECT_DOUBLE_EQ(c.poses.front().x,.02);
    EXPECT_LE(c.length,8.); for(const auto & p:c.poses) EXPECT_TRUE(safe(map,body(),p,true));
  }
  map.setCost(160,160,254);
  auto blocked=search(map,body(),{.02,.02,.036},limits,Clock::now()+std::chrono::seconds(1));
  EXPECT_TRUE(blocked.curves.empty()); EXPECT_EQ(blocked.evaluated,0u);
}
