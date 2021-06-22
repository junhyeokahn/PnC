FILE(GLOB mpc_sources *.cpp)
FILE(GLOB mpc_headers "*.hpp")

#find_package(osqp REQUIRED)
#include_directories(/usr/local/include/osqp/)
include_directories(/home/knapsack/project/locomotion/src/PnC_june/PnC/A1PnC/)

add_library(myMPC SHARED ${mpc_sources})

target_link_libraries(myMPC myUtils
                            # myGaitCycle
                            # myGoldfarb
                            myRobotSystem
                            myOSQP)

add_executable(test_cmpc test_cmpc.cpp)
target_link_libraries(test_cmpc  myMPC
#                                  myGaitCycle
                                 myUtils
                                 myGoldfarb
                                 myRobotSystem
                                 myOSQP)
# add_executable(run_test_mpc test_mpc.cpp)
# target_link_libraries(run_test_mpc myUtils
#                                    myGaitCycle
#                                    myGoldfarb
#                                    myRobotSystem)


# add_executable(test_mpc_trajectory_manager test_mpc_trajectory_manager.cpp)
# target_link_libraries(test_mpc_trajectory_manager  myMPC
#                                                    myGaitCycle
#                                                    myUtils
#                                                    myGoldfarb
#                                                    myRobotSystem)


# add_executable(run_test_mpc_gait_cycle run_test_mpc_gait_cycle.cpp)
# target_link_libraries(run_test_mpc_gait_cycle  myMPC
#                                                myGaitCycle
#                                                myUtils
#                                                myGoldfarb
#                                                myRobotSystem)

install(TARGETS myMPC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${mpc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/ConvexMPC")
