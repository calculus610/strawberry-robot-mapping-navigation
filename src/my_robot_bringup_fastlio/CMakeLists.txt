cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup_fastlio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_qos_relay
  src/scan_qos_relay.cpp
)

ament_target_dependencies(scan_qos_relay
  rclcpp
  sensor_msgs
)

add_executable(height_projection_map
  src/height_projection_map.cpp
)

ament_target_dependencies(height_projection_map
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(rack_obstacle_layer
  src/rack_obstacle_layer.cpp
)

ament_target_dependencies(rack_obstacle_layer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(map_autosaver
  src/map_autosaver.cpp
)

ament_target_dependencies(map_autosaver
  nav_msgs
  rclcpp
  std_srvs
)

add_executable(cmd_vel_shutdown_guard
  src/cmd_vel_shutdown_guard.cpp
)

ament_target_dependencies(cmd_vel_shutdown_guard
  geometry_msgs
  rclcpp
)

install(TARGETS
  rack_obstacle_layer
  height_projection_map
  cmd_vel_shutdown_guard
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

ament_environment_hooks(
  env-hooks/zzj_nav2_isolation.sh
)

ament_python_install_package(manual_cloud)
install(PROGRAMS scripts/manual_cloud_filter.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
