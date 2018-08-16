file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(DracoHipPnC SHARED ${srcs})
target_link_libraries(DracoHipPnC drake::drake
                                  ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  myWBC)
