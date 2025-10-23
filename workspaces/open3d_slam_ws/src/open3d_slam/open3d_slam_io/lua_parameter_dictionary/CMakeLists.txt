cmake_minimum_required(VERSION 3.12)
project(lua_parameter_dictionary)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Lua REQUIRED)

set(Unwind_INCLUDE_DIR "/usr/include")
set(Unwind_LIBRARY "/usr/lib/x86_64-linux-gnu/libunwind.so")

find_package(glog REQUIRED)
find_package(gflags REQUIRED)

# Always print the include path for debugging
message(STATUS "LUA_INCLUDE_DIR: ${LUA_INCLUDE_DIR}")

include_directories(
  include
  ${LUA_INCLUDE_DIR}           # This is critical!
  ${Boost_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
  ${gflags_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
   src/lua_parameter_dictionary.cc
   src/configuration_file_resolver.cc
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${LUA_LIBRARIES}
  glog::glog
  gflags
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# *** Export system includes for downstream packages ***
ament_export_include_directories(
  include
  ${LUA_INCLUDE_DIR}
)
ament_export_libraries(${PROJECT_NAME})

ament_package()
