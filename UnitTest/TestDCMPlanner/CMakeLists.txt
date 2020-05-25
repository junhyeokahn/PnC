add_executable(test_footstep_object test_footstep_object.cpp)
target_link_libraries(test_footstep_object ${DART_LIBRARIES}
                                                          myRobotSystem
                                                          myUtils
                                                          myWBC
                                                          myFilters
                                                          myPlanner
                                                          gtest
                                                          gtest_main
                                                          pthread)

