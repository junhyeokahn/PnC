file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(PendulumPnC SHARED ${srcs})
target_link_libraries(PendulumPnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils)
