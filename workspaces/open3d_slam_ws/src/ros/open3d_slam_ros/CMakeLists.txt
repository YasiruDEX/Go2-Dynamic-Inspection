cmake_minimum_required(VERSION 3.16)
project(open3d_slam_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")

# ---------------------------------------------------------------------------
# 1-a  Compiler flags
# ---------------------------------------------------------------------------
add_compile_options(
  -O3 -mtune=native -flto -funroll-loops -fopenmp
  -fopt-info-vec-optimized -fno-math-errno)

# ---------------------------------------------------------------------------
# 1-b  Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

set(Unwind_INCLUDE_DIR "/usr/include")
set(Unwind_LIBRARY "/usr/lib/x86_64-linux-gnu/libunwind.so")


find_package(Glog REQUIRED)

include_directories(
  include
  ${GLOG_INCLUDE_DIRS}
)

# Link directories for Glog (specify where the glog library resides)
link_directories(/lib/x86_64-linux-gnu)

find_package(yaml-cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)



find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)

find_package(open3d_conversions REQUIRED)
find_package(open3d_slam REQUIRED)
find_package(open3d_slam_lua_io REQUIRED)

# ---------------------------------------------------------------------------
# 1-c  Include paths
# ---------------------------------------------------------------------------
include_directories(
  include
  ${GLOG_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 1-d  Library
# ---------------------------------------------------------------------------
set(SRC_FILES
  src/helpers_ros.cpp
  src/SlamMapInitializer.cpp
  src/SlamWrapperRos.cpp
  src/OnlineRangeDataProcessorRos.cpp
  src/creators.cpp
  src/DataProcessorRos.cpp
  src/AiryProcessorRos.cpp
  # src/Propagator.cpp
  src/RobosenseProcessorRos.cpp
  src/RosbagRangeDataProcessorRos.cpp
  src/Color.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs
  geometry_msgs nav_msgs sensor_msgs visualization_msgs rosgraph_msgs
  interactive_markers
  open3d_conversions open3d_slam open3d_slam_lua_io
  Glog yaml-cpp Eigen3
  rosbag2_cpp rosbag2_storage rosbag2_storage_default_plugins
  std_srvs
  tf2_geometry_msgs tf2_eigen
)

# ONLY use the plain signature here:
target_link_libraries(${PROJECT_NAME} yaml-cpp ${GLOG_LIBRARIES} -lglog)
target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
target_compile_definitions(${PROJECT_NAME} PUBLIC open3d_slam_ros_OPENMP_FOUND=1)

# ---------------------------------------------------------------------------
# 1-e  Executables
# ---------------------------------------------------------------------------
add_executable(mapping_node src/mapping_node.cpp)
ament_target_dependencies(mapping_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs
  geometry_msgs nav_msgs sensor_msgs visualization_msgs rosgraph_msgs
  interactive_markers
  open3d_conversions open3d_slam open3d_slam_lua_io
  Glog yaml-cpp Eigen3
  rosbag2_cpp rosbag2_storage rosbag2_storage_default_plugins std_srvs tf2_geometry_msgs tf2_eigen rclcpp
)
target_link_libraries(mapping_node ${PROJECT_NAME} yaml-cpp ${GLOG_LIBRARIES} -lglog)
target_link_libraries(mapping_node OpenMP::OpenMP_CXX)
set_target_properties(mapping_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

add_executable(airy_processing_node src/airy_processing_node.cpp)
ament_target_dependencies(airy_processing_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs
  geometry_msgs nav_msgs sensor_msgs visualization_msgs rosgraph_msgs
  interactive_markers
  open3d_conversions open3d_slam open3d_slam_lua_io
  Glog yaml-cpp Eigen3
  rosbag2_cpp rosbag2_storage rosbag2_storage_default_plugins std_srvs tf2_geometry_msgs tf2_eigen
)
target_link_libraries(airy_processing_node ${PROJECT_NAME} yaml-cpp ${GLOG_LIBRARIES} -lglog)
target_link_libraries(airy_processing_node OpenMP::OpenMP_CXX)
set_target_properties(airy_processing_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

add_executable(robosense_processing_node src/robosense_processing_node.cpp)
ament_target_dependencies(robosense_processing_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs
  geometry_msgs nav_msgs sensor_msgs visualization_msgs rosgraph_msgs
  interactive_markers
  open3d_conversions open3d_slam open3d_slam_lua_io
  Glog yaml-cpp Eigen3
  rosbag2_cpp rosbag2_storage rosbag2_storage_default_plugins std_srvs tf2_geometry_msgs tf2_eigen
)
target_link_libraries(robosense_processing_node ${PROJECT_NAME} yaml-cpp ${GLOG_LIBRARIES} -lglog)
target_link_libraries(robosense_processing_node OpenMP::OpenMP_CXX)
set_target_properties(robosense_processing_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# add_executable(propagator_processing_node src/propagator_processing_node.cpp)
# ament_target_dependencies(propagator_processing_node
#   rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_msgs
#   geometry_msgs nav_msgs sensor_msgs visualization_msgs rosgraph_msgs
#   interactive_markers
#   open3d_conversions open3d_slam open3d_slam_lua_io
#   Glog yaml-cpp Eigen3
#   rosbag2_cpp rosbag2_storage rosbag2_storage_default_plugins std_srvs tf2_geometry_msgs tf2_eigen
# )
# target_link_libraries(propagator_processing_node ${PROJECT_NAME} yaml-cpp ${GLOG_LIBRARIES} -lglog)
# target_link_libraries(propagator_processing_node OpenMP::OpenMP_CXX)
# set_target_properties(propagator_processing_node PROPERTIES
#   BUILD_WITH_INSTALL_RPATH TRUE
#   INSTALL_RPATH_USE_LINK_PATH TRUE
# )

# ---------------------------------------------------------------------------
# 1-f  Installation
# ---------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}
  mapping_node
  airy_processing_node
  robosense_processing_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # Ensures executables go to libexec
)

add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)

# Install ROS2 package.xml for ament
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
