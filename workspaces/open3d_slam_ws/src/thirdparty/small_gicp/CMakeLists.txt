cmake_minimum_required(VERSION 3.16)
project(small_gicp VERSION 1.0.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --------------------------------------------------------------------------- #
# Interface target (header-only)
# --------------------------------------------------------------------------- #
add_library(small_gicp INTERFACE)

# Include directory of small_gicp
target_include_directories(small_gicp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(OpenMP_FOUND)
  target_link_libraries(small_gicp INTERFACE OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(small_gicp INTERFACE Eigen3)

# --------------------------------------------------------------------------- #
# Install headers and libraries
# --------------------------------------------------------------------------- #
install(DIRECTORY include/ DESTINATION include)

# --------------------------------------------------------------------------- #
# Export targets
# --------------------------------------------------------------------------- #
install(TARGETS small_gicp EXPORT small_gicp-targets)
ament_export_targets(small_gicp-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
