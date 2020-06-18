file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")
file(GLOB draco_logic_interrupt_headers "DracoLogicInterrupt/*.hpp")
file(GLOB draco_ctrl_headers "DracoCtrl/*.hpp")
file(GLOB draco_state_estimator_headers "DracoStateEstimator/*.hpp")
file(GLOB draco_state_machine_headers "DracoStateMachine/*.hpp")
file(GLOB draco_task_headers "DracoTask/*.hpp")
file(GLOB draco_ctrl_arch_headers "DracoCtrlArchitecture/*.hpp")
file(GLOB draco_taf_headers "DracoTaskAndForceContainer/*.hpp")

add_library(DracoPnC SHARED ${sources})
target_link_libraries(DracoPnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC
                               myFilters
                               myPlanner
                               myTrajectoryManager)

install(TARGETS DracoPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC")

install(FILES ${draco_logic_interrupt_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoLogicInterrupt")
install(FILES ${draco_ctrl_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoCtrl")
install(FILES ${draco_state_estimator_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoStateEstimator")
install(FILES ${draco_state_machine_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoStateMachine")
install(FILES ${draco_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoTask")
install(FILES ${draco_ctrl_arch_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoCtrlArchitecture")
install(FILES ${draco_taf_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/DracoTaskAndForceContainer")
