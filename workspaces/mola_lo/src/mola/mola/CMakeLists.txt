cmake_minimum_required(VERSION 3.5)

project(mola)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# This is a meta package, there are no targets.
# Install files:
install(DIRECTORY
  #rviz2
  DESTINATION
    share/${PROJECT_NAME}
)

# Silent warning on unused variable:
set(ignored ${CMAKE_EXPORT_COMPILE_COMMANDS})
unset(ignored)
