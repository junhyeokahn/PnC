SET(MPC_SOURCES CMPC.cpp)
SET(MPC_HEADERS CMPC.hpp)

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


install(TARGETS myMPC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${MPC_HEADERS} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/MPC")
