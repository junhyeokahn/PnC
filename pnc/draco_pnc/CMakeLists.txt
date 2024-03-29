file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")
file(GLOB draco_task_headers "draco_task/*.hpp")
file(GLOB draco_state_machine_headers "draco_state_machine/*.hpp")

add_library(pnc-draco-pnc SHARED ${sources})
target_link_libraries(pnc-draco-pnc ${DART_LIBRARIES}
                                    ${ZMQ_LIBRARIES}
                                    ${Protobuf_LIBRARIES}
                                    pnc-msg-draco
                                    pnc-robot-system
                                    pnc-utils
                                    pnc-filters
                                    pnc-dcm-planner
                                    pnc-wbc)

install(TARGETS pnc-draco-pnc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/draco_pnc")
install(FILES ${draco_state_machine_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/draco_pnc/draco_state_machine")
install(FILES ${draco_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/draco_pnc/draco_task")
