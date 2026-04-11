# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                              (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_sm_loop_closure LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)


# find dependencies:
find_package(GTSAM REQUIRED)
find_package(mola_georeferencing REQUIRED)
find_package(mola_gtsam_factors REQUIRED)
find_package(mola_pose_list REQUIRED)
find_package(mola_relocalization REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mp2p_icp_filters REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)

# -----------------------
# define lib:
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    module/src/SimplemapLoopClosure.cpp
    module/src/FrameToFrameLoopClosure.cpp
    module/src/LoopClosureInterface.cpp
    module/src/register.cpp
    # Headers:
    module/include/mola_sm_loop_closure/SimplemapLoopClosure.h
    module/include/mola_sm_loop_closure/FrameToFrameLoopClosure.h
  PUBLIC_LINK_LIBRARIES
    gtsam
    mola::mola_georeferencing
    mola::mola_gtsam_factors
    mola::mola_pose_list
    mola::mola_relocalization
    mola::mola_yaml
    mola::mp2p_icp
    mola::mp2p_icp_filters
    mrpt::gui
    mrpt::maps
#	PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mola_georeferencing
    mola_gtsam_factors
    mola_pose_list
    mola_relocalization
    mp2p_icp
    mp2p_icp_filters
    mrpt-maps
)

# mola_add_library() expects headers under "ROOT/include", so do this manually here:
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# CLI apps:
# ---------------------------------
mola_add_executable(
  TARGET  mola-sm-lc-cli
  SOURCES apps/mola-sm-lc-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::maps
    mrpt::tclap
)


# -----------------------
# define tests:
#enable_testing()
#add_subdirectory(module/tests)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  ament_package()
endif()
