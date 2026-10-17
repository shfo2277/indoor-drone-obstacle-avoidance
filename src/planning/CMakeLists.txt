cmake_minimum_required(VERSION 3.5)
project(planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cannyedge_test1_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(planning_node src/planning.cpp)
target_include_directories(planning_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(planning_node
  rclcpp
  px4_msgs
  geometry_msgs
  sensor_msgs
  cannyedge_test1_interfaces
  octomap_msgs
  nav_msgs
  pcl_conversions
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
  PCL
)

install(TARGETS planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

