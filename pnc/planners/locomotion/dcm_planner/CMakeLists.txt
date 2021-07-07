FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(pnc-dcm-planner ${sources})
target_link_libraries(pnc-dcm-planner pnc-utils)

install(TARGETS pnc-dcm-planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/planners/locomotion/dcm_planner")
