add_subdirectory(WBC)
add_subdirectory(Planner)
add_subdirectory(RobotSystem)
add_subdirectory(Filter)
add_subdirectory(AtlasPnC)
add_subdirectory(draco_pnc)

file(GLOB pnc_headers "*.hpp")
install(FILES ${pnc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC")
