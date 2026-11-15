cmake_minimum_required(VERSION 3.10)
project(robot_base_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Hardware / protocol library (no ROS dependencies)
# ---------------------------------------------------------------------------
add_library(base_hw
  src/serial_port.cpp
  src/ddsm115_protocol.cpp
  src/bno085.cpp
  src/ibus.cpp)

target_include_directories(base_hw
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# ---------------------------------------------------------------------------
# Nodes
# ---------------------------------------------------------------------------
add_executable(base_controller_node src/base_controller_node.cpp)
target_link_libraries(base_controller_node base_hw)
ament_target_dependencies(base_controller_node
  rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(bno085_imu_node src/bno085_imu_node.cpp)
target_link_libraries(bno085_imu_node base_hw)
ament_target_dependencies(bno085_imu_node
  rclcpp sensor_msgs)

add_executable(flysky_ibus_node src/flysky_ibus_node.cpp)
target_link_libraries(flysky_ibus_node base_hw)
ament_target_dependencies(flysky_ibus_node
  rclcpp geometry_msgs)

add_executable(odom_fusion_node src/odom_fusion_node.cpp)
target_link_libraries(odom_fusion_node Eigen3::Eigen)
ament_target_dependencies(odom_fusion_node
  rclcpp std_srvs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  base_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  base_controller_node
  bno085_imu_node
  flysky_ibus_node
  odom_fusion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(base_hw)
ament_package()
