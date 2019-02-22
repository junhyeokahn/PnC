add_subdirectory(DracoPnC)
add_subdirectory(WBC)
add_subdirectory(PlannerSet)
add_subdirectory(CartPolePnC)

file(GLOB pnc_headers "*.hpp")
install(FILES ${pnc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC")
