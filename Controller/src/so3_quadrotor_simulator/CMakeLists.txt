cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_ros2.cpp)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  ament_index_cpp
  quadrotor_msgs
)

install(TARGETS quadrotor_simulator_so3 quadrotor_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch include config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
