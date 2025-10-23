cmake_minimum_required(VERSION 3.12)
project(open3d_conversions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(
  -O3 -std=c++17 -mtune=native -funroll-loops -fopenmp
  -fopt-info-vec-optimized -fno-math-errno
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(open3d_slam_msgs REQUIRED)
find_package(Open3D REQUIRED)

if(Open3D_FOUND)
  message(STATUS "Found Open3D: ${Open3D_DIR}")
else()
  message(FATAL_ERROR "Open3D not found. Aborting build.")
endif()

add_library(open3d_conversions src/open3d_conversions.cpp)

target_include_directories(open3d_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(open3d_conversions
  Open3D::Open3D
)

ament_target_dependencies(open3d_conversions
  rclcpp
  sensor_msgs
  Eigen3
  std_msgs
  open3d_slam_msgs
  Open3D
)

install(TARGETS open3d_conversions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(open3d_conversions)
ament_export_dependencies(rclcpp sensor_msgs Eigen3 std_msgs open3d_slam_msgs Open3D)

ament_package()
