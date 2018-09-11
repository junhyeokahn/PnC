add_executable(test_drake_dart TestDrakeDart.cpp)
target_link_libraries(test_drake_dart drake::drake
                                      ${DART_LIBRARIES}
                                      myRobotSystem
                                      myUtils
                                      gtest
                                      gtest_main)
add_test(NAME test_drake_dart COMMAND test_drake_dart)

add_executable(test_inv_kin TestInvKin.cpp)
target_link_libraries(test_inv_kin drake::drake
                                   ${DART_LIBRARIES}
                                   myRobotSystem
                                   myUtils
                                   gtest
                                   gtest_main)
add_test(NAME test_inv_kin COMMAND test_inv_kin)

add_executable(test_draco TestDraco.cpp)
target_link_libraries(test_draco drake::drake
                                 ${DART_LIBRARIES}
                                 myRobotSystem
                                 myUtils
                                 gtest
                                 gtest_main)
add_test(NAME test_draco COMMAND test_draco)

add_executable(test_traj TestTrajectory.cpp)
target_link_libraries(test_traj drake::drake
                                ${DART_LIBRARIES}
                                myRobotSystem
                                myUtils
                                gtest
                                gtest_main)
add_test(NAME test_traj COMMAND test_traj)
