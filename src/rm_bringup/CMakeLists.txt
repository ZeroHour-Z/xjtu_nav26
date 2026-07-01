cmake_minimum_required(VERSION 3.8)
project(rm_bringup)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(base_link_ground_projector src/base_link_ground_projector.cpp)
ament_target_dependencies(base_link_ground_projector
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS base_link_ground_projector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config PCD rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
