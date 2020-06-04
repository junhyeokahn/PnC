add_subdirectory(ValkyriePnC)
add_subdirectory(WBC)
add_subdirectory(Planner)
add_subdirectory(TrajectoryManager)
add_subdirectory(RobotSystem)
add_subdirectory(Filter)

file(GLOB pnc_headers "*.hpp")
install(FILES ${pnc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC")
