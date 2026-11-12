cmake_minimum_required(VERSION 3.16)
project(strawberry_octomap VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(octomap_server 2.3.1 EXACT CONFIG REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(nav2_map_server REQUIRED)

set(server_header_dependencies
  message_filters nav_msgs octomap_msgs octomap_ros pcl_conversions pcl_ros
  rcl_interfaces rclcpp sensor_msgs std_msgs std_srvs tf2_ros visualization_msgs)
foreach(dependency IN LISTS server_header_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

# The upstream package exports several variants that compile the same class.
# Link only the ordinary XYZ server and the Nav2 map I/O library.
find_library(OCTOMAP_SERVER_LIBRARY NAMES octomap_server
  PATHS "${octomap_server_DIR}/../../../lib" NO_DEFAULT_PATH REQUIRED)
find_library(NAV2_MAP_IO_LIBRARY NAMES map_io
  PATHS "${nav2_map_server_DIR}/../../../lib" NO_DEFAULT_PATH REQUIRED)

add_executable(occupancy_server src/occupancy_server.cpp)
target_compile_features(occupancy_server PRIVATE cxx_std_17)
target_include_directories(occupancy_server SYSTEM PRIVATE
  ${octomap_server_INCLUDE_DIRS} ${nav2_map_server_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_definitions(occupancy_server PRIVATE
  STRAWBERRY_PACKAGE_VERSION="${PROJECT_VERSION}"
  STRAWBERRY_OCTOMAP_SERVER_VERSION="${octomap_server_VERSION}"
  STRAWBERRY_OCTOMAP_VERSION="${OCTOMAP_VERSION}"
  STRAWBERRY_NAV2_MAP_SERVER_VERSION="${nav2_map_server_VERSION}")
ament_target_dependencies(occupancy_server ${server_header_dependencies})
target_link_libraries(occupancy_server
  ${OCTOMAP_SERVER_LIBRARY} ${NAV2_MAP_IO_LIBRARY}
  ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS occupancy_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

ament_package()
