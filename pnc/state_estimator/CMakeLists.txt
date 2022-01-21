FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(pnc-state-estimator ${sources} ${headers})

target_link_libraries(pnc-state-estimator pnc-utils)

install(TARGETS pnc-state-estimator DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/pnc/state_estimator")