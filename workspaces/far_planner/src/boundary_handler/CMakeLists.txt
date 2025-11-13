cmake_minimum_required(VERSION 3.8)

# CUDA 12.6 supports GCC-11, no need to set host compiler explicitly
project(boundary_handler LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)

# Add C++17 compile features
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find CUDA
find_package(CUDAToolkit REQUIRED)
include_directories(${CUDAToolkit_INCLUDE_DIRS})

# Set CUDA architectures for common GPUs
# Adjust these based on your target GPU
# 75 = RTX 2000 series, 80 = A100, 86 = RTX 3000 series
set(CMAKE_CUDA_ARCHITECTURES 75)

# CUDA compilation flags to handle C++ standard library compatibility
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --expt-extended-lambda")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC")

# Separate host and device compilation
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
  /usr/include
)

link_directories(
  /usr/local/lib
  /usr/lib
)

set(SOURCES
  # Add your source files here if any
)

# CUDA source files
set(CUDA_SOURCES
  src/graph_cuda_kernels.cu
  src/graph_cuda_wrapper.cpp
)

add_executable(${PROJECT_NAME} src/graph_extractor.cpp ${SOURCES} ${CUDA_SOURCES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  visibility_graph_msg
)

# Make sure to link against the PCL libraries and any other libraries you need
target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
  CUDA::cudart
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()