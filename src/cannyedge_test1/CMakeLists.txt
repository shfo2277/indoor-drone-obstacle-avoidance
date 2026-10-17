cmake_minimum_required(VERSION 3.5)
project(cannyedge_test1)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "cannyedge_test1/msg/HoleInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_python_install_package(${PROJECT_NAME})
ament_package()

