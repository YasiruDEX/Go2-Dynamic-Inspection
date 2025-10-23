cmake_minimum_required(VERSION 3.12)
project(open3d_slam)

# --------------------------------------------------------------------------- #
# 0. Compiler & options
# --------------------------------------------------------------------------- #
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(
  -O3 -mtune=native -flto -funroll-loops -fopenmp
  -fopt-info-vec-optimized -fno-math-errno)

option(ENABLE_PROFILING "Enable ProfilerScopeGuard instrumentation" OFF)
if(ENABLE_PROFILING)
  add_compile_definitions(ENABLE_PROFILING)
endif()

# --------------------------------------------------------------------------- #
# 1. Source files
# --------------------------------------------------------------------------- #
set(SRC_FILES
  src/helpers.cpp               src/time.cpp          src/math.cpp
  src/Odometry.cpp              src/Voxel.cpp         src/croppers.cpp
  src/Submap.cpp                src/SubmapCollection.cpp
  src/Transform.cpp             src/TransformInterpolationBuffer.cpp
  src/AdjacencyMatrix.cpp       src/PlaceRecognition.cpp
  src/OptimizationProblem.cpp   src/SlamWrapper.cpp
  src/constraint_builders.cpp   src/output.cpp
  src/MotionCompensation.cpp    src/VoxelHashMap.cpp
  src/ScanToMapRegistration.cpp src/CloudRegistration.cpp
)


list(PREPEND SRC_FILES src/Mapper.cpp)

# --------------------------------------------------------------------------- #
# 2. Dependencies
# --------------------------------------------------------------------------- #
find_package(ament_cmake REQUIRED)
find_package(Eigen3        REQUIRED)
find_package(Boost         REQUIRED COMPONENTS serialization filesystem system program_options)
# find_package(TBB           REQUIRED)
find_package(OpenMP REQUIRED)  # Ensure OpenMP is found
find_package(yaml-cpp      REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(small_gicp    REQUIRED)
find_package(rclcpp        REQUIRED)  # Added for ROS 2 message types
find_package(Open3D        REQUIRED)

if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  add_definitions(-Dopen3d_slam_OPENMP_FOUND=1)
else()
  message(FATAL_ERROR "OpenMP not found. Please install OpenMP development files.")
endif()

# --------------------------------------------------------------------------- #
# 3. Library target
# --------------------------------------------------------------------------- #
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $<$<BOOL:${OpenMP_CXX_INCLUDE_DIRS}>:${OpenMP_CXX_INCLUDE_DIRS}>
)

# Embed shared library install locations as RPATH to find at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# --------------------------------------------------------------------------- #
# 4. Linking with libraries (use keyword signature consistently)
# --------------------------------------------------------------------------- #
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${Open3D_LIBRARIES}
    small_gicp::small_gicp
    yaml-cpp
  PRIVATE
    ${Boost_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# Use keyword signature for all target dependencies
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    open3d_conversions
    nav_msgs
    geometry_msgs
    rclcpp                     # Ensure rclcpp is linked for ROS 2 message types
    Eigen3
    Boost
    yaml-cpp
    small_gicp
    OpenMP  # Add OpenMP as a dependency here for `open3d_slam` only
    Open3D
)

# --------------------------------------------------------------------------- #
# 5. Install & Export
# --------------------------------------------------------------------------- #
install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  open3d_conversions nav_msgs geometry_msgs
  Eigen3 Boost yaml-cpp small_gicp rclcpp OpenMP Open3D
)

ament_package()
