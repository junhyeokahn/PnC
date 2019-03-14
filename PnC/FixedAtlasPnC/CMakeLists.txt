file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(FixedAtlasPnC ${sources})
target_link_libraries(FixedAtlasPnC ${DART_LIBRARIES}
                                    myRobotSystem
                                    myUtils)
