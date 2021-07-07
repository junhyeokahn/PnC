file(GLOB sources "*.cpp" "*.c")
file(GLOB robot_system_headers "*.hpp")

add_library(pnc-robot-system ${sources}
                             ${robot_system_headers})

target_link_libraries(pnc-robot-system ${DART_LIBRARIES}
                                       pnc-utils)

install(TARGETS pnc-robot-system DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${robot_system_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/robot_system")
