cmake_minimum_required(VERSION 3.8)
project(turtle_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(turtle_fusion_node src/turtle_fusion_node.cpp
)

ament_target_dependencies(turtle_fusion_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
)


install(TARGETS
  turtle_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
