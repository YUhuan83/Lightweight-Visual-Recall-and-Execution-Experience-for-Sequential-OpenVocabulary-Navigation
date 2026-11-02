cmake_minimum_required(VERSION 3.5)
project(grid_map_cv)

## Find ament packages
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED
  COMPONENTS
    photo
)

grid_map_package()

## Specify additional locations of header files
include_directories(
  include
)

set(dependencies
  cv_bridge
  filters
  grid_map_core
  OpenCV
  pluginlib
  rclcpp
  sensor_msgs
)

###########
## Build ##
###########

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  src/GridMapCvProcessing.cpp
  src/InpaintFilter.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  cv_bridge::cv_bridge
  grid_map_core::grid_map_core
  opencv_photo
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Filters does not expose targets (yet)
# https://github.com/ros/filters/pull/70
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${filters_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation.
install(
  FILES filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############


if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(filters filter_plugins.xml)
ament_package()
