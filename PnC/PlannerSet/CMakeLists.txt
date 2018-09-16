file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

add_library(myPlanner SHARED ${srcs})

target_link_libraries(myPlanner myUtils
                                myContact
                                myConicSolver
                                myRobotSystem)
