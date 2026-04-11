# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

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

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_state_estimation_smoother LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt-obs)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

# Find MOLA packages:
find_package(mola_kernel REQUIRED)
find_package(mola_imu_preintegration REQUIRED)
find_package(mola_gtsam_factors REQUIRED)

# -----------------------
# define lib:
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    src/Parameters.cpp
    src/StateEstimationSmoother.cpp
    src/pose_pdf_to_string_with_sigmas.cpp
    src/register.cpp
    include/mola_state_estimation_smoother/Parameters.h
    include/mola_state_estimation_smoother/StateEstimationSmoother.h
    include/mola_state_estimation_smoother/pose_pdf_to_string_with_sigmas.h
  PUBLIC_LINK_LIBRARIES
    mrpt::obs
    mola::mola_kernel
    mola::mola_imu_preintegration
    mola::mola_gtsam_factors
  PRIVATE_LINK_LIBRARIES
    gtsam
    gtsam_unstable
  CMAKE_DEPENDENCIES
    mola_common
    mola_imu_preintegration
    mola_gtsam_factors
    mola_kernel
)
target_include_directories(${PROJECT_NAME} PRIVATE ".")

# Install files:
install(
  DIRECTORY
    mola-cli-launchs
    ros2-launchs
    params
  DESTINATION
    share/${PROJECT_NAME}
)

# -----------------------
# define cli apps:
mola_add_executable(
  TARGET  mola-navstate-cli
  SOURCES apps/mola-navstate-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
)

if(DETECTED_ROS2)
  # -----------------------------------------------------------------------------
  #  ROS2
  # -----------------------------------------------------------------------------
  # find dependencies
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    add_subdirectory(tests)
  endif()

  ament_package()
else()
  # define tests (w/o ROS)
  enable_testing()
  add_subdirectory(tests)
endif()

