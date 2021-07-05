cmake_minimum_required(VERSION 3.5.1)
project(PnC)

##=============================================================================
## CMake Flags
##=============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS on)
set(CXX_DISABLE_WERROR True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

##=============================================================================
## Build Setup
##=============================================================================
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib )
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin )
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
set(PYTHON_BINDING TRUE)

##=============================================================================
## Include Directory
##=============================================================================
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

##=============================================================================
## Dependancy
##=============================================================================

# dart
find_package(DART 6.11 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

# eigen
if(NOT EIGEN_FOUND)
  find_package(Eigen)
endif()
include_directories(${EIGEN_INCLUDE_DIRS})

# zmq and protobuf
set(B_MESSAGING false)
find_package(ZMQ)
find_package(Protobuf)
if(ZMQ_FOUND AND Protobuf_FOUND)
  include_directories(${ZMQ_INCLUDE_DIRS})
  include_directories(${Protobuf_INCLUDE_DIRS})
  add_subdirectory(messages)
  set(B_MESSAGING true)
endif()

# pybind11
if(PYTHON_BINDING)
  find_package(Python COMPONENTS Interpreter Development)
  find_package(pybind11 CONFIG)
  include_directories(${pybind11_INCLUDE_DIRS})
  add_subdirectory(bindings)
endif()


##=============================================================================
## Add Subdirectory
##=============================================================================
add_subdirectory(Simulator)
add_subdirectory(Utils)
add_subdirectory(ExternalSource)
add_subdirectory(PnC)
add_subdirectory(test)

##=============================================================================
## Configuration File
##=============================================================================
set(THIS_COM "${PROJECT_SOURCE_DIR}/" )
set(SERVO_RATE 0.001)
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake
    ${PROJECT_SOURCE_DIR}/Configuration.hpp)
install(FILES ${PROJECT_SOURCE_DIR}/Configuration.hpp DESTINATION "${INSTALL_INCLUDE_DIR}")
