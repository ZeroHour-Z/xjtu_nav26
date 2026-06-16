cmake_minimum_required(VERSION 3.8)
project(rm_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/rm_communication
)

add_library(rm_comm_protocol INTERFACE)
target_include_directories(rm_comm_protocol INTERFACE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/rm_communication
)

add_executable(serial_rw_node src/serial_rw_node.cpp)
target_link_libraries(serial_rw_node rm_comm_protocol)
ament_target_dependencies(serial_rw_node rclcpp std_msgs)

add_executable(virtual_serial_node src/virtual_serial_node.cpp)
target_link_libraries(virtual_serial_node rm_comm_protocol)
ament_target_dependencies(virtual_serial_node rclcpp std_msgs)

add_executable(handler_node src/handler_node.cpp)
target_link_libraries(handler_node rm_comm_protocol)
ament_target_dependencies(handler_node rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(aim_udp_bridge_node src/aim_udp_bridge_node.cpp)
ament_target_dependencies(aim_udp_bridge_node rclcpp std_msgs)

install(TARGETS
  serial_rw_node
  virtual_serial_node
  handler_node
  aim_udp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_control_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package() 
