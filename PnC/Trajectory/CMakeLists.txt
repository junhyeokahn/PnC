FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(myTrajectory SHARED ${sources} ${headers})
target_link_libraries(myTrajectory myUtils
                                   ${DART_LIBRARIES})
