cmake_minimum_required(VERSION 3.5.1)
project(PnC)

##=============================================================================
## CMake Flags
##=============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS off)
set(CXX_DISABLE_WERROR True)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

##=============================================================================
## Build Setup
##=============================================================================
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib )
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin )
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})

##=============================================================================
## Dependancy
##=============================================================================

# System
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

# Dart
find_package(DART 6.9 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

# Eigen
if(NOT EIGEN_FOUND)
  find_package(Eigen3)
endif()
include_directories(${EIGEN3_INCLUDE_DIRS})

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

##=============================================================================
## Include Directory
##=============================================================================
include_directories("${PROJECT_SOURCE_DIR}")

##=============================================================================
## Add Subdirectory
##=============================================================================
add_subdirectory(Simulator)
add_subdirectory(Utils)
add_subdirectory(ExternalSource)
add_subdirectory(PnC)

set(USE_PYBIND11 True)
if(USE_PYBIND11)
    message("--Enable Pybind11--")
    find_package(PythonAnaconda)
    find_package(pybind11 CONFIG REQUIRED)
    include_directories(/home/knapsack/software/anaconda3/envs/mingyo/include)
    include_directories(/home/knapsack/software/anaconda3/envs/mingyo/include/pybind11)
    add_subdirectory(Bindings)
else()
    message("--Disable Pybind11--")
endif()

##=============================================================================
## Configuration File
##=============================================================================
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
install(FILES ${PROJECT_SOURCE_DIR}/Configuration.h DESTINATION "${INSTALL_INCLUDE_DIR}")
