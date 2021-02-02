project (Supervisor)
cmake_minimum_required (VERSION 3.5.1)

set (${PROJECT_NAME}_VERSION_NAME "star")
set (${PROJECT_NAME}_VERSION_MAJOR 1)
set (${PROJECT_NAME}_VERSION_MINOR 0)
set (${PROJECT_NAME}_VERSION_PATCH 0)
set (${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
if(NOT EIGEN_FOUND)
  find_package(Eigen3)
endif()
include_directories(${EIGEN3_INCLUDE_DIRS})


message ("")
message ("-[ MAKE INFORMATION ]-----------------------------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message ("--------------------------------------------------------------------------------")
message ("")

include_directories ("${PROJECT_SOURCE_DIR}/../../Utils")
include_directories ("${PROJECT_SOURCE_DIR}/../../ExternalSource")
include_directories ("${PROJECT_SOURCE_DIR}/../../")

set (headers
DataSave.h
data_protocol.h
../../Utils/IO/Pthread.hpp
../../Utils/IO/comm_udp.hpp
../../Utils/IO/IOUtilities.hpp
../../Utils/IO/DataManager.hpp
)

set (sources
DataSave.cpp
Status_Display.cpp
../../Utils/IO/Pthread.cpp
../../Utils/IO/comm_udp.cpp
../../Utils/IO/IOUtilities.cpp
../../Utils/IO/DataManager.cpp
)

#link_directories("/usr/local/lib/")
#link_directories("/usr/lib/")

add_executable(Status_Display ${sources} ${headers})
# add_dependencies(Status_Display comm_comm)
target_link_libraries(Status_Display pthread)
