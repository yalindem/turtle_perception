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
find_package(Eigen3 REQUIRED)

add_executable(turtle_yaw_ekf_node src/turtle_yaw_ekf_node.cpp)
target_link_libraries (turtle_yaw_ekf_node Eigen3::Eigen)

ament_target_dependencies(turtle_yaw_ekf_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
)


install(TARGETS
  turtle_yaw_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
