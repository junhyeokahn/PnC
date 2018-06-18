file(GLOB_RECURSE srcs "*.cpp" "*.hpp")
add_library(myRobotSystem SHARED ${srcs})

target_link_libraries(myRobotSystem ${DART_LIBRARIES})
