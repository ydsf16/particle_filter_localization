cmake_minimum_required(VERSION 2.8.3)
project(particle_filter_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  pcl_ros
)


catkin_package(
)

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3  REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/grid_map.cc
src/motion_model.cc
src/measurement_model.cc
src/localizer.cc
)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
)


add_executable(odometry
test/odometry.cc)
target_link_libraries(odometry
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


add_executable(pf_localization_node
test/pf_localization_node.cc)
target_link_libraries(pf_localization_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)