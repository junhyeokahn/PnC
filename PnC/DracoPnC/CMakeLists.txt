file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(DracoPnC SHARED ${srcs})
target_link_libraries(DracoPnC drake::drake
                               ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC)
