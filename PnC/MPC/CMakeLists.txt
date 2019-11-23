SET(MPC_SOURCES CMPC.cpp)

add_library(myMPC SHARED ${MPC_SOURCES})
target_link_libraries(myMPC myUtils
                            myGoldfarb
                            myRobotSystem)

add_executable(run_test_mpc_toy run_test_mpc_toy.cpp)
target_link_libraries(run_test_mpc_toy  myMPC
                                        myUtils
                                        myGoldfarb
                                        myRobotSystem)

add_executable(run_test_mpc test_mpc.cpp)
target_link_libraries(run_test_mpc myUtils
                            	   myGoldfarb
                            	   myRobotSystem)
