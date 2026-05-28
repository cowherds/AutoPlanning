cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(CUDA REQUIRED)
find_package(pcl_conversions REQUIRED)

set(YOPO_CUDA_ARCH "" CACHE STRING "Force CUDA arch number, e.g. 86")
set(YOPO_CUDA_ARCH_FLAGS "" CACHE STRING "Force raw NVCC arch flags")

if(NOT "${YOPO_CUDA_ARCH_FLAGS}" STREQUAL "")
  set(ARCH_FLAGS "${YOPO_CUDA_ARCH_FLAGS}")
elseif(NOT "${YOPO_CUDA_ARCH}" STREQUAL "")
  set(ARCH_FLAGS "-gencode arch=compute_${YOPO_CUDA_ARCH},code=sm_${YOPO_CUDA_ARCH}")
else()
  cuda_select_nvcc_arch_flags(ARCH_FLAGS)
endif()

if("${ARCH_FLAGS}" STREQUAL "")
  message(FATAL_ERROR "Failed to decide CUDA architecture flags. Set -DYOPO_CUDA_ARCH=86 (or your arch).")
endif()

if("${ARCH_FLAGS}" MATCHES "compute_89")
  message(WARNING "Detected compute_89, but your nvcc may not support it. Fallback to compute_86.")
  set(ARCH_FLAGS "-gencode arch=compute_86,code=sm_86")
endif()

message(STATUS "Using CUDA arch flags: ${ARCH_FLAGS}")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${ARCH_FLAGS}")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

cuda_add_library(raycast_cuda src/sensor_simulator.cu)
target_link_libraries(raycast_cuda ${CUDA_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(sensor_simulator src/sensor_simulator_ros2.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(sensor_simulator
  raycast_cuda
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  OpenMP::OpenMP_CXX
)
target_compile_definitions(sensor_simulator PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")
ament_target_dependencies(sensor_simulator
  rclcpp
  nav_msgs
  sensor_msgs
  cv_bridge
  pcl_conversions
)

add_executable(dataset_generator src/dataset_generator.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(dataset_generator
  raycast_cuda
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
target_compile_definitions(dataset_generator PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")

install(TARGETS sensor_simulator dataset_generator raycast_cuda
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
