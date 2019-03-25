file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(AtlasPnC SHARED ${sources})
target_link_libraries(AtlasPnC ${DART_LIBRARIES}
                                myRobotSystem
                                myUtils
                                myFilters
                                myPlanner
                                myWBC)
