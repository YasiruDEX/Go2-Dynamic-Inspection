cmake_minimum_required(VERSION 3.12)
project(open3d_slam_lua_io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

set(SRC_FILES
  src/parameter_loaders.cpp
  src/LuaLoader.cpp
)

find_package(ament_cmake REQUIRED)
find_package(open3d_slam REQUIRED)
find_package(lua_parameter_dictionary REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  open3d_slam
  lua_parameter_dictionary
  ament_index_cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(open3d_slam lua_parameter_dictionary ament_index_cpp)
ament_package()
