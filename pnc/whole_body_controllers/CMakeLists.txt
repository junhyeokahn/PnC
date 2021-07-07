FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers "*.hpp")
FILE(GLOB ihwbc_headers "ihwbc/*.hpp")
FILE(GLOB manager_headers "managers/*.hpp")

add_library(pnc-wbc ${sources})

target_link_libraries(pnc-wbc pnc-utils
                              pnc-goldfarb
                              pnc-robot-system
                              pnc-dcm-planner)

install(TARGETS pnc-wbc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/whole_body_controllers")
install(FILES ${ihwbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/whole_body_controllers/ihwbc")
install(FILES ${manager_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/whole_body_controllers/managers")
