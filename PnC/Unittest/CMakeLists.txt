add_executable(test_kinematics TestKinematics.cpp)
target_link_libraries(test_kinematics drake::drake
                                      ${DART_LIBRARIES}
                                      myRobotSystem
                                      myUtils
                                      gtest gtest_main)
add_test(NAME test_kinematics COMMAND test_kinematics)
