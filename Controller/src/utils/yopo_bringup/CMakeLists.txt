cmake_minimum_required(VERSION 3.8)
project(yopo_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/test_super_compat.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
