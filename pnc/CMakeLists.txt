add_subdirectory(whole_body_controllers)
add_subdirectory(planners)
add_subdirectory(robot_system)
add_subdirectory(filters)
add_subdirectory(atlas_pnc)
add_subdirectory(draco_pnc)

file(GLOB pnc_headers "*.hpp")
install(FILES ${pnc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/pnc")
