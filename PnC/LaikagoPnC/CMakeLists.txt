file(GLOB_RECURSE sources "*.cpp")
file(GLOB laikago_pnc_headers "*.hpp")
file(GLOB laikago_logic_interrupt_headers "LaikagoLogicInterrupt/*.hpp")
file(GLOB laikago_ctrl_headers "LaikagoCtrl/*.hpp")
file(GLOB laikago_state_estimator_headers "LaikagoStateEstimator/*.hpp")
file(GLOB laikago_state_machine_headers "LaikagoStateMachine/*.hpp")
file(GLOB laikago_task_headers "LaikagoTask/*.hpp")
file(GLOB laikago_ctrl_arch_headers "LaikagoCtrlArchitecture/*.hpp")
file(GLOB laikago_taf_headers "LaikagoTaskAndForceContainer/*.hpp")

add_library(LaikagoPnC SHARED ${sources})
target_link_libraries(LaikagoPnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myFilters
                               myPlanner
                               myTrajectoryManager)

install(TARGETS LaikagoPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${laikago_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC")

install(FILES ${laikago_logic_interrupt_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoLogicInterrupt")
install(FILES ${laikago_ctrl_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoCtrl")
install(FILES ${laikago_state_estimator_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoStateEstimator")
install(FILES ${laikago_state_machine_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoStateMachine")
install(FILES ${laikago_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoTask")
install(FILES ${laikago_ctrl_arch_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoCtrlArchitecture")
install(FILES ${laikago_taf_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/LaikagoPnC/LaikagoTaskAndForceContainer")
