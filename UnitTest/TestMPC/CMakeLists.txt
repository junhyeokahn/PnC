add_executable(test_walking_reference_trajectories test_walking_reference_trajectories.cpp)
target_link_libraries(test_walking_reference_trajectories DracoPnC ${DART_LIBRARIES}
                                                          myRobotSystem
                                                          myUtils
                                                          myMPC
                                                          myGaitCycle
                                                          myWBC
                                                          myFilters
                                                          myPlanner
                                                          gtest
                                                          gtest_main
                                                          pthread)


add_executable(test_dcm_reference test_dcm_reference.cpp)
target_link_libraries(test_dcm_reference DracoPnC ${DART_LIBRARIES}
                                                          myRobotSystem
                                                          myUtils
                                                          myMPC
                                                          myGaitCycle
                                                          myWBC
                                                          myFilters
                                                          myPlanner
                                                          gtest
                                                          gtest_main
                                                          pthread)
