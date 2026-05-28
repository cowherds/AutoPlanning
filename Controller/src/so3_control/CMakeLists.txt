cmake_minimum_required(VERSION 3.8)
project(so3_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(network_control_node
  src/network_control_ros2.cpp
  src/SO3Control.cpp
)

ament_target_dependencies(network_control_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  quadrotor_msgs
)

install(TARGETS network_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
