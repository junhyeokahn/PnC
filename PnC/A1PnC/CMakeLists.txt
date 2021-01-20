file(GLOB_RECURSE sources "*.cpp")
file(GLOB a1_pnc_headers "*.hpp")

add_library(A1PnC SHARED ${sources})
target_link_libraries(A1PnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC)

install(TARGETS A1PnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${a1_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC")

