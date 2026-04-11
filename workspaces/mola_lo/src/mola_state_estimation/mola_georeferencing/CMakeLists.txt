# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                              (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake version: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(default_build_type "Release")
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "Coverage")
# Set flags for the "Coverage" build type. This is robust for use with colcon.
set(CMAKE_CXX_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-update=atomic" CACHE STRING "Flags for C++ coverage build")
set(CMAKE_C_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-update=atomic" CACHE STRING "Flags for C coverage build")
set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "--coverage" CACHE STRING "Linker flags for coverage build")
set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "--coverage" CACHE STRING "Linker flags for coverage build")


if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_georeferencing LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
#find_package(CLI11 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(mola_common REQUIRED)
find_package(mola_gtsam_factors REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)

# -----------------------
# define lib:
# -----------------------
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    src/simplemap_georeference.cpp
    include/mola_georeferencing/simplemap_georeference.h
  PUBLIC_LINK_LIBRARIES
    gtsam
    mola::mola_gtsam_factors
    mola::mola_yaml
    mola::mp2p_icp
    mrpt::maps
#	PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mp2p_icp
    mrpt-maps

)

# ---------------------------------
# CLI apps:
# ---------------------------------
mola_add_executable(
  TARGET  mola-sm-georeferencing
  SOURCES apps/mola-sm-georeferencing-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::maps
    mrpt::tclap
    #CLI11::CLI11
)

mola_add_executable(
  TARGET  mola-trajectory-georef
  SOURCES apps/mola-trajectory-georef-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::maps
    mrpt::tclap
    #CLI11::CLI11
)

mola_add_executable(
  TARGET  mola-mm-add-geodetic
  SOURCES apps/mola-mm-add-geodetic-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::maps
    mrpt::tclap
    #CLI11::CLI11
)

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  ament_package()
endif()
