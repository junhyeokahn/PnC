file(GLOB_RECURSE sources "*.cpp")
file(GLOB scorpio_pnc_headers "*.hpp")
file(GLOB scorpio_testset_headers "TestSet/*.hpp")
file(GLOB scorpio_taskset_headers "TaskSet/*.hpp")
file(GLOB scorpio_ctrlset_headers "CtrlSet/*.hpp")

add_library(Scorpio2PnC SHARED ${sources})
target_link_libraries(Scorpio2PnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC)

install(TARGETS Scorpio2PnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${scorpio2_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Scorpio2PnC")
install(FILES ${scorpio2_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Scorpio2PnC/TestSet")
install(FILES ${scorpio2_ctrlset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Scorpio2PnC/CtrlSet")
install(FILES ${scorpio2_taskset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Scorpio2PnC/TaskSet")
