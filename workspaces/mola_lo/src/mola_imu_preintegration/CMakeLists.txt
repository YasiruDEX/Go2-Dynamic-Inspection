# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_imu_preintegration LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt-obs)

# Find MOLA packages:
#find_mola_package(mola-kernel)

# -----------------------
# define lib:
set(LIB_SRCS
  src/ImuInitialCalibrator.cpp
  src/ImuIntegrationParams.cpp
  src/ImuIntegrator.cpp
  src/ImuTransformer.cpp
  src/LocalVelocityBuffer.cpp
  src/register.cpp
  src/trajectory_from_buffer.cpp
)
set(LIB_PUBLIC_HDRS
  include/mola_imu_preintegration/ImuInitialCalibrator.h
  include/mola_imu_preintegration/ImuIntegrationParams.h
  include/mola_imu_preintegration/ImuIntegrator.h
  include/mola_imu_preintegration/ImuTransformer.h
  include/mola_imu_preintegration/LocalVelocityBuffer.h
  include/mola_imu_preintegration/trajectory_from_buffer.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::containers
    mrpt::poses
    mrpt::obs
  CMAKE_DEPENDENCIES
    mola_common
)

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)
