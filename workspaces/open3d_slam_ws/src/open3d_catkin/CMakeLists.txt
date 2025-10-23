cmake_minimum_required(VERSION 3.24)
cmake_policy(SET CMP0135 NEW)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

project(open3d_catkin)

# ────────────────────────────────────────────────────────────────────────────────
# Catkin and dependencies
# ────────────────────────────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS roscpp roslib)
set(CATKIN_PACKAGE_DEPENDENCIES roscpp roslib)

# ────────────────────────────────────────────────────────────────────────────────
# Options and configuration
# ────────────────────────────────────────────────────────────────────────────────
option(OPEN3D_FROM_SOURCE "Build Open3D from source instead of using prebuilt version" OFF)

set(OPEN3D_VERSION 0.19.0)
set(OPEN3D_TAG v${OPEN3D_VERSION})
set(OPEN3D_URL https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VERSION}/open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_VERSION}.tar.xz)
set(OPEN3D_HASH 2e525fd2afe7e80907d2b6e3c66b69e2ef1481dbceed5727b749a6e65cba5720)

# Check if Open3D can be found
find_package(Open3D ${OPEN3D_VERSION} QUIET)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldMagenta "${Esc}[1;35m")
endif()

# ────────────────────────────────────────────────────────────────────────────────
# 1. Use manually specified Open3D_DIR
# ────────────────────────────────────────────────────────────────────────────────
if(DEFINED ENV{Open3D_DIR})
  set(Open3D_DIR $ENV{Open3D_DIR})
  find_package(Open3D CONFIG REQUIRED)
  message("${BoldMagenta}INFO: Found Open3D via environment variable at $ENV{Open3D_DIR}.${ColourReset}")
  set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../include/")
  find_package(fmt REQUIRED)

  catkin_package(
    INCLUDE_DIRS
      ${Open3D_INCLUDE_DIRS}
      ${Open3D_INCLUDE_DIR_MAN}
      ${fmt_INCLUDE_DIRS}
    LIBRARIES
      ${Open3D_LIBRARIES}
      fmt::fmt
    CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
    DEPENDS Open3D
  )
# ────────────────────────────────────────────────────────────────────────────────
# 2. Check if Open3D is installed
# ────────────────────────────────────────────────────────────────────────────────
elseif(Open3D_FOUND)
  message("${BoldMagenta}INFO: Found Open3D installation at ${Open3D_DIR}.${ColourReset}")
  find_package(fmt REQUIRED)

  catkin_package(
    INCLUDE_DIRS
        ${Open3D_INCLUDE_DIRS}
        ${fmt_INCLUDE_DIRS}
    LIBRARIES
        ${Open3D_LIBRARIES}
        fmt::fmt
    CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
    DEPENDS Open3D
  )
# ────────────────────────────────────────────────────────────────────────────────
# 3. Use prebuilt binary (default)
# ────────────────────────────────────────────────────────────────────────────────
elseif(NOT OPEN3D_FROM_SOURCE)
  include(FetchContent)

  set(OPEN3D_DOWNLOAD_DIR ${CMAKE_SOURCE_DIR}/prebuilt)
  set(OPEN3D_SOURCE_DIR   ${CMAKE_SOURCE_DIR}/prebuilt/open3d-${OPEN3D_VERSION})
  set(Open3D_DIR          ${OPEN3D_SOURCE_DIR}/lib/cmake/Open3D)

  if(NOT EXISTS "${Open3D_DIR}/Open3DConfig.cmake")
    message("${BoldMagenta}INFO: Fetching prebuilt Open3D ${OPEN3D_VERSION} from ${OPEN3D_URL}${ColourReset}")
    FetchContent_Declare(open3d_prebuilt
      URL         ${OPEN3D_URL}
      URL_HASH    SHA256=${OPEN3D_HASH}
      DOWNLOAD_DIR ${OPEN3D_DOWNLOAD_DIR}
      SOURCE_DIR   ${OPEN3D_SOURCE_DIR}
    )
    FetchContent_MakeAvailable(open3d_prebuilt)
  else()
    message("${BoldMagenta}INFO: Found existing Open3D at ${OPEN3D_SOURCE_DIR}, skipping download.${ColourReset}")
  endif()

  find_package(Open3D REQUIRED CONFIG)
  find_package(fmt REQUIRED)

  set(Open3D_INCLUDE_WRAPPER ${CATKIN_DEVEL_PREFIX}/include)
  file(MAKE_DIRECTORY ${Open3D_INCLUDE_WRAPPER}/open3d)
  file(GLOB_RECURSE O3D_HEADERS "${OPEN3D_SOURCE_DIR}/include/open3d/*.h")
  foreach(hdr IN LISTS O3D_HEADERS)
    file(RELATIVE_PATH rel_path "${OPEN3D_SOURCE_DIR}/include" "${hdr}")
    configure_file("${hdr}" "${Open3D_INCLUDE_WRAPPER}/${rel_path}" COPYONLY)
  endforeach()

  catkin_package(
    INCLUDE_DIRS
      ${Open3D_INCLUDE_WRAPPER}
      ${fmt_INCLUDE_DIRS}
    LIBRARIES
      ${Open3D_LIBRARIES}
      fmt::fmt
    CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
    DEPENDS Open3D
  )

# ────────────────────────────────────────────────────────────────────────────────
# 4. Build from source if requested
# ────────────────────────────────────────────────────────────────────────────────
else()
  message("${BoldMagenta}INFO: Building Open3D ${OPEN3D_VERSION} from source.${ColourReset}")

  include(ExternalProject)
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

  ExternalProject_Add(open3d
    GIT_REPOSITORY https://github.com/isl-org/Open3D
    GIT_TAG        ${OPEN3D_TAG}
    GIT_SUBMODULES_RECURSE TRUE
    GIT_PROGRESS   TRUE
    PREFIX         "${CMAKE_SOURCE_DIR}"
    TMP_DIR        "${CMAKE_BINARY_DIR}/open3d/tmp"
    BINARY_DIR     "${CMAKE_BINARY_DIR}/open3d/build"
    INSTALL_DIR    "${CATKIN_DEVEL_PREFIX}"
    CMAKE_ARGS
      -DBUILD_SHARED_LIBS=ON
      -DCMAKE_BUILD_TYPE=Release
      -DGLIBCXX_USE_CXX11_ABI=ON
      -DUSE_SYSTEM_EIGEN3=OFF
      -DBUILD_PYTHON_MODULE=OFF
      -DBUILD_EXAMPLES=OFF
      -DBUILD_GUI=OFF
      -DBUILD_WEBRTC=OFF
      -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
      "-DCMAKE_CXX_FLAGS_RELEASE=-std=c++17 -O3 -mtune=native -flto -funroll-loops -fopenmp -fno-math-errno"
      "-DCMAKE_C_FLAGS_RELEASE=-O3 -mtune=native -flto -funroll-loops -fopenmp -fno-math-errno"
  )

  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES    ${CATKIN_DEVEL_PREFIX}/lib/libOpen3D.so
    CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )
  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
    DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  )
endif()
