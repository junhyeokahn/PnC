file(GLOB_RECURSE sources "*.cpp")
file(GLOB a1_pnc_headers "*.hpp")
file(GLOB a1_logic_interrupt_headers "A1LogicInterrupt/*.hpp")
file(GLOB a1_state_estimator_headers "A1StateEstimator/*.hpp")
file(GLOB a1_taf_headers "A1TaskAndForceContainer/*.hpp")
file(GLOB a1_ctrl_arch_headers "A1CtrlArchitecture/*.hpp")
file(GLOB a1_task_headers "A1Task/*.hpp")
file(GLOB a1_state_machine_headers "A1StateMachine/*.hpp")
file(GLOB a1_ctrl_headers "A1Ctrl/*.hpp")


add_library(A1PnC SHARED ${sources})
target_link_libraries(A1PnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myTrajectoryManager
                               myFilters
                               myPlanner
                               myMPC)

install(TARGETS A1PnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${a1_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC")


install(FILES ${a1_logic_interrupt_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1LogicInterrupt")
install(FILES ${a1_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1Task")
install(FILES ${a1_taf_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1TaskAndForceContainer")
install(FILES ${a1_state_estimator_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1StateEstimator")
install(FILES ${a1_ctrl_arch_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1CtrlArchitecture")
install(FILES ${a1_state_machine_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1StateMachine")
install(FILES ${a1_ctrl_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/A1Ctrl")
