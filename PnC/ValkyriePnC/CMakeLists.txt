file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(ValkyriePnC ${sources})
target_link_libraries(ValkyriePnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  myFilters
                                  myPlanner
                                  myWBC)
