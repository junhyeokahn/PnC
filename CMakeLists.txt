cmake_minimum_required(VERSION 3.5.1)
project(PnC)

## ===========
## CMake Flags
## ===========
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CXX_DISABLE_WERROR True)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

## ==========
## Dependancy
## ==========

# Dart
find_package(DART 6.5 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

# Drake
find_package(drake CONFIG REQUIRED)

# Eigen
#find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})

# Gurobi
find_package(Gurobi)
include_directories(${GUROBI_INCLUDE_DIRS})

# Mosek
find_package(Mosek)
include_directories(${MOSEK_INCLUDE_DIRS})

## =================
## Include Directory
## =================

## ================
## Add Subdirectory
## ================

## ==================
## Configuration File
## ==================

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
