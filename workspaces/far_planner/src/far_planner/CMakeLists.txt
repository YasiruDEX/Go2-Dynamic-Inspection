cmake_minimum_required(VERSION 3.8)

# CUDA 12.6 supports GCC-11, no need to set host compiler explicitly
project(far_planner LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 and CUDA standard required
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)

# Find CUDA
find_package(CUDAToolkit REQUIRED)
include_directories(${CUDAToolkit_INCLUDE_DIRS})

# Set CUDA architectures for common GPUs
set(CMAKE_CUDA_ARCHITECTURES 75)

# CUDA compilation flags
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --expt-extended-lambda")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC")

# Separate host and device compilation
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Build
set(SOURCES ${SOURCES}
  src/dynamic_graph.cpp
  src/planner_visualizer.cpp
  src/utility.cpp
  src/graph_planner.cpp
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/map_handler.cpp
  src/scan_handler.cpp
  src/graph_msger.cpp
  src/terrain_planner.cpp
)

# CUDA source files
set(CUDA_SOURCES
  src/graph_cuda_collision.cu
  src/far_planner_cuda_wrapper.cpp
)

add_executable(${PROJECT_NAME} src/far_planner.cpp ${SOURCES} ${CUDA_SOURCES})

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs sensor_msgs nav_msgs visualization_msgs pcl_conversions visibility_graph_msg tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} CUDA::cudart)

# Waypoint to Cmd_Vel converter
add_executable(waypoint_to_cmd_vel src/waypoint_to_cmd_vel.cpp)
ament_target_dependencies(waypoint_to_cmd_vel rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS ${PROJECT_NAME} waypoint_to_cmd_vel DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

ament_package()
