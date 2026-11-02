cmake_minimum_required(VERSION 3.8)
project(graphnav_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pose_graph_msgs REQUIRED)
find_package(graphnav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nanoflann REQUIRED)


add_library(graphnav_mapper_component SHARED
  src/mapper.cpp
  src/mapper_node.cpp
  src/navigation_graph.cpp
)
target_include_directories(graphnav_mapper_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(graphnav_mapper_component
  rclcpp
  rclcpp_components
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pose_graph_msgs
  graphnav_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
)
target_link_libraries(graphnav_mapper_component
  ${OpenCV_LIBRARIES}
  nanoflann::nanoflann
)
rclcpp_components_register_node(graphnav_mapper_component PLUGIN "graphnav_mapper::MapperNode" EXECUTABLE mapper_node)


install(TARGETS graphnav_mapper_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dummy_pose_graph_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
