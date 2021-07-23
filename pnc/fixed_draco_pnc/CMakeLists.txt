file(GLOB_RECURSE sources "*.cpp")
file(GLOB fixed_draco_pnc_headers "*.hpp")
file(GLOB fixed_draco_state_machine_headers "fixed_draco_state_machine/*.hpp")

add_library(pnc-fixed-draco-pnc SHARED ${sources})
target_link_libraries(pnc-fixed-draco-pnc ${DART_LIBRARIES}
                                    ${ZMQ_LIBRARIES}
                                    ${Protobuf_LIBRARIES}
                                    pnc-msg-fixed-draco
                                    pnc-robot-system
                                    pnc-utils
                                    pnc-filters
                                    pnc-wbc)

install(TARGETS pnc-fixed-draco-pnc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${fixed_draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/fixed_draco_pnc")
install(FILES ${fixed_draco_state_machine_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/fixed_draco_pnc/fixed_draco_state_machine")
