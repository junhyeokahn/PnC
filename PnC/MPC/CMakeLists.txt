
add_executable(run_test_mpc test_mpc.cpp)

target_link_libraries(run_test_mpc myUtils
                            	   myGoldfarb
                            	   myRobotSystem)
