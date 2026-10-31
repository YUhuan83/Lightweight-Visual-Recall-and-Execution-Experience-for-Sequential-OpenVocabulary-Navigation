"""Build a diagnostic-only variant before any production modification."""
from pathlib import Path
import zipfile
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
base=(out/'diagnostic_planner.reference.cpp').read_text(encoding='utf-8-sig')
base=base.replace('TEST_F(ActualPlanner, FullTrialRecordedGeometryAudit)', 'TEST_F(ActualPlanner, FullTrialRecordedGeometryAudit)')
old='  configure();\n  const std::string label="case"'
extra='''  configure();
  std::cout<<"PENALTIES "<<node->get_parameter("CarLike.change_penalty").as_double()
           <<","<<node->get_parameter("CarLike.reverse_penalty").as_double()<<std::endl;
  Limits lim;lim.radius=12.;
  for(int variant=0;variant<2;variant++) {
    const auto begin=Clock::now();
    const auto deadline=begin+std::chrono::milliseconds(200);
    auto result=variant?candidateSearch(*grid,map->getRobotFootprint(),start,lim,deadline):
      search(*grid,map->getRobotFootprint(),start,lim,deadline);
    std::cout<<"CONNECTOR "<<index<<","<<variant<<","<<result.evaluated<<","<<result.timed_out
      <<","<<std::chrono::duration<double>(Clock::now()-begin).count()<<std::endl;
    for(unsigned k=0;k<result.curves.size();k++) {
      const auto & c=result.curves[k];
      std::cout<<"CURVE "<<index<<","<<variant<<","<<k<<","<<c.length<<","<<c.cusps<<std::endl;
    }
  }
  const std::string label="case"'''
assert old in base
base=base.replace(old,extra)
marker='using namespace wildos_ackermann_smac;'
base=base.replace(marker,marker+'\nnamespace wildos_ackermann_smac { SearchResult candidateSearch(nav2_costmap_2d::Costmap2D &, const nav2_costmap_2d::Footprint &, Pose, const Limits &, Clock::time_point); }')
(out/'diagnostic_planner.cpp').write_text(base,encoding='utf8')
source=(root/'ackermann_smac/src/connector.cpp').read_text(encoding='utf8')
function=source[source.index('SearchResult search('):].replace('SearchResult search(', 'SearchResult candidateSearch(',1)
old='''          ++out.evaluated;
          if (checker.inCollision(x, y, bin, lim.allow_unknown)) { continue; }
          // Match NodeHybrid float angle output, including wrap-around precision.
          float angle = bin * (2.f * static_cast<float>(M_PI) / lim.bins);'''
new='''          // Any connector accepted by connect has length <= radius, hence
          // its net yaw change cannot exceed one radian at curvature <= 1/radius.
          float angle = bin * (2.f * static_cast<float>(M_PI) / lim.bins);
          if (std::abs(std::remainder(angle-from.yaw,2*M_PI)) > 1.+1e-6) { continue; }
          ++out.evaluated;
          if (checker.inCollision(x, y, bin, lim.allow_unknown)) { continue; }'''
assert old in function
function=function.replace(old,new)
prefix=source[:source.index('namespace wildos_ackermann_smac {')+len('namespace wildos_ackermann_smac {')]
(out/'connector_variant.cpp').write_text(prefix+'\n'+function,encoding='utf8')
cmake='''cmake_minimum_required(VERSION 3.16)
project(connector_closed_audit)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ompl REQUIRED)
foreach(dep wildos_ackermann_smac nav2_smac_planner nav2_core nav2_costmap_2d nav2_util rclcpp rclcpp_lifecycle pluginlib geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
  find_package(${dep} REQUIRED)
endforeach()
ament_add_gtest(recorded_planner diagnostic_planner.cpp connector_variant.cpp)
target_include_directories(recorded_planner PRIVATE ${OMPL_INCLUDE_DIRS})
target_compile_definitions(recorded_planner PRIVATE FIXTURE_DIR="/home/ly/wildos-console/bundle/ackermann_smac/test/fixtures")
ament_target_dependencies(recorded_planner wildos_ackermann_smac nav2_smac_planner nav2_core nav2_costmap_2d nav2_util rclcpp rclcpp_lifecycle pluginlib geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
'''
(out/'CMakeLists.txt').write_text(cmake,encoding='utf8')
with zipfile.ZipFile(out/'diagnostic.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name in ('diagnostic_planner.cpp','connector_variant.cpp','CMakeLists.txt'):
        z.write(out/name,name)
