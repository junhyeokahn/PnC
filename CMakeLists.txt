cmake_minimum_required(VERSION 3.5.1)
project(PnC)

## ===========
## CMake Flags
## ===========
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CXX_DISABLE_WERROR True)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

## ==========
## Dependancy
## ==========
# Dart
find_package(DART 6.4 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

# Drake
set(drake_DIR /opt/drake/lib/cmake/drake)
find_package(drake CONFIG REQUIRED)

# Eigen
if(NOT EIGEN_FOUND)
  find_package(Eigen)
endif()
include_directories(${EIGEN_INCLUDE_DIRS})

# Gurobi
find_package(Gurobi)
if(GUROBI_FOUND)
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# Mosek
find_package(Mosek)
if(MOSEK_FOUND)
  include_directories(${MOSEK_INCLUDE_DIRS})
endif()

## =================
## Include Directory
## =================
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/Simulator")
include_directories("${PROJECT_SOURCE_DIR}/RobotSystem")
include_directories("${PROJECT_SOURCE_DIR}/Utils")
include_directories("${PROJECT_SOURCE_DIR}/PnC")
include_directories("${PROJECT_SOURCE_DIR}/PnC/Contact")
include_directories("${PROJECT_SOURCE_DIR}/PnC/WBC")
include_directories("${PROJECT_SOURCE_DIR}/PnC/CentroidPlanner")
include_directories("${PROJECT_SOURCE_DIR}/PnC/WBC/WBLC")
include_directories("${PROJECT_SOURCE_DIR}/PnC/Trajectory")
include_directories("${PROJECT_SOURCE_DIR}/ExternalSource/Yaml/include")
include_directories("${PROJECT_SOURCE_DIR}/ExternalSource")
include_directories("${PROJECT_SOURCE_DIR}/ExternalSource/Optimizer/")
include_directories("${PROJECT_SOURCE_DIR}/ExternalSource/Optimizer/ConicSolver/include")
include_directories("${PROJECT_SOURCE_DIR}/Geometry/Polytope")

## ================
## Add Subdirectory
## ================
add_subdirectory(Simulator)
add_subdirectory(RobotSystem)
add_subdirectory(Utils)
add_subdirectory(ExternalSource)
add_subdirectory(PnC)
add_subdirectory(Geometry)

## ==================
## Configuration File
## ==================
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
