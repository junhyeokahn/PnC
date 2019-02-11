add_subdirectory(DracoPnC)
add_subdirectory(WBC)
add_subdirectory(PlannerSet)
add_subdirectory(CartPolePnC)
add_subdirectory(NeuralNetwork)

file(GLOB pnc_headers "*.hpp")
install(FILES ${pnc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC")
