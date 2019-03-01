file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(FixedDracoPnC ${sources})
target_link_libraries(FixedDracoPnC ${DART_LIBRARIES}
                                    myRobotSystem
                                    myUtils)
