// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include "wildos_ackermann_smac/path_contract.hpp"
using namespace wildos_ackermann_smac;
std::vector<Pose> fixture(const std::string & name) {
  std::ifstream input(std::string(FIXTURE_DIR)+"/"+name);std::string line;
  if(!input.good()) throw std::runtime_error("Missing fixture "+name);
  std::getline(input,line);std::vector<Pose> path;
  while(std::getline(input,line)) {
    std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);Pose p;
    if(!(row>>p.x>>p.y>>p.yaw)) throw std::runtime_error("Bad fixture "+name);
    path.push_back(p);
  }
  return path;
}
TEST(PathContract, RecordedRawAndSmoothedCuspDiffer) {
  for(int i:{0,1,3}) {
    SCOPED_TRACE(i);
    auto raw=fixture("full_trial_case"+std::to_string(i)+"_raw.csv");
    auto smooth=fixture("full_trial_case"+std::to_string(i)+"_smooth.csv");
    for(double radius:{8.,12.}) {
      EXPECT_TRUE(curvatureFeasible(raw,.25,radius,72));
      EXPECT_FALSE(curvatureFeasible(smooth,.25,radius,72));
    }
  }
}
TEST(PathContract, CuspCannotCancelSharpTerminalHeading) {
  std::vector<Pose> path{{0,0,0},{1,0,0},{2,0,0},{3,0,.2},{2,0,0},{1,0,0},{0,0,0}};
  EXPECT_FALSE(curvatureFeasible(path,.25,8.,0));
  path[3].yaw=0;EXPECT_TRUE(curvatureFeasible(path,.25,8.,0));
  path.insert(path.begin()+3,path[2]);EXPECT_TRUE(curvatureFeasible(path,.25,8.,0));
}
TEST(PathContract, ArcBoundAndCoordinateTransform) {
  for(double radius:{8.,7.99}) {
    std::vector<Pose> path;
    for(int i=0;i<=12;i++) { double h=i*.04;path.push_back({radius*std::sin(h),radius*(1-std::cos(h)),h}); }
    EXPECT_EQ(curvatureFeasible(path,.25,8.,0),radius==8.);
    for(auto & p:path) { double x=p.x;p.x=734.-p.y;p.y=-12.+x;p.yaw+=std::acos(-1.)/2.; }
    EXPECT_EQ(curvatureFeasible(path,.25,8.,0),radius==8.);
    std::reverse(path.begin(),path.end());
    EXPECT_EQ(curvatureFeasible(path,.25,8.,0),radius==8.);
  }
}
TEST(PathContract, InvalidInputFailsClosed) {
  EXPECT_FALSE(curvatureFeasible({},.25,8.,72));
  EXPECT_FALSE(curvatureFeasible({{0,0,0},{1,0,0}},.25,0.,72));
  EXPECT_FALSE(curvatureFeasible({{0,0,0},{1,0,std::numeric_limits<double>::quiet_NaN()}},.25,8.,72));
  EXPECT_FALSE(curvatureFeasible({{0,0,0},{1,0,0}},0.,8.,72));
}
