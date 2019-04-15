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
find_package(DART 6.8 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

# Zmq
find_package(ZMQ)
if(ZMQ_FOUND)
    include_directories(${ZMQ_INCLUDE_DIRS})
endif()

# Protobuf
find_package(Protobuf)
if(Protobuf_FOUND)
    include_directories(${Protobuf_INCLUDE_DIRS})
endif()

# Pybind11
set(USE_PYBIND11 False)
if(USE_PYBIND11)
  message("-- Enable Pybind11")
  find_package(PythonAnaconda)
else()
  message("-- Disable Pybind11")
endif()

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

# GTest
find_package(GTest)
if(GTEST_FOUND)
    enable_testing()
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
if(Protobuf_FOUND AND ZMQ_FOUND)
    add_subdirectory(Messages)
endif()
if(USE_PYBIND11)
  add_subdirectory(Bindings)
endif()
if(GTEST_FOUND)
  add_subdirectory(UnitTest)
endif()
add_subdirectory(ReinforcementLearning)


##=============================================================================
## Configuration File
##=============================================================================
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
if (ZMQ_FOUND AND Protobuf_FOUND)
    SET (HAS_RL_DEP true)
else()
    SET (HAS_RL_DEP false)
endif()
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
install(FILES ${PROJECT_SOURCE_DIR}/Configuration.h DESTINATION "${INSTALL_INCLUDE_DIR}")
