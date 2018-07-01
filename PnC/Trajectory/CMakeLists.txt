file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(myTrajectory SHARED ${srcs})
target_link_libraries(myTrajectory  myUtils
                                    ${DART_LIBRARIES})
