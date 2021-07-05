file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")
file(GLOB draco_state_machine_headers "draco_state_machine/*.hpp")

add_library(pnc-draco-pnc ${sources})
target_link_libraries(pnc-draco-pnc ${DART_LIBRARIES}
                                    myRobotSystem
                                    pnc-util
                                    myFilters
                                    myDCMPlanner
                                    myWBC)

install(TARGETS pnc-draco-pnc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/draco_pnc")
install(FILES ${draco_state_machine_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/draco_pnc/draco_state_machine")
