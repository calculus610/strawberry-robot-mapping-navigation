cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup_fastlio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(scan_qos_relay
  src/scan_qos_relay.cpp
)

ament_target_dependencies(scan_qos_relay
  rclcpp
  sensor_msgs
)

add_executable(map_autosaver
  src/map_autosaver.cpp
)

ament_target_dependencies(map_autosaver
  nav_msgs
  rclcpp
  std_srvs
)

install(TARGETS
  map_autosaver
  scan_qos_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
