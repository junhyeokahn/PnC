file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(AtlasPnC ${sources})
target_link_libraries(AtlasPnC ${DART_LIBRARIES}
                               myRobotSystem
                               pnc-util
                               myFilters
                               myDCMPlanner
                               myWBC)
