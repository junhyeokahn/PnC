add_subdirectory(DracoPnC)
add_subdirectory(WBC)
add_subdirectory(PlannerSet)

if(ZMQ_FOUND AND Protobuf_FOUND)
    add_subdirectory(CartPolePnC)
    add_subdirectory(ReinforcementLearning)
endif()

file(GLOB pnc_headers "*.hpp")
install(FILES ${pnc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/PnC")
