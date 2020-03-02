file(GLOB_RECURSE sources "*.cpp")
file(GLOB scorpio_pnc_headers "*.hpp")
file(GLOB scorpio_testset_headers "TestSet/*.hpp")
file(GLOB scorpio_taskset_headers "TaskSet/*.hpp")
file(GLOB scorpio_ctrlset_headers "CtrlSet/*.hpp")

add_library(ScorpioPnC SHARED ${sources})
target_link_libraries(ScorpioPnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC)

install(TARGETS ScorpioPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${scorpio_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/ScorpioPnC")
install(FILES ${scorpio_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/ScorpioPnC/TestSet")
install(FILES ${scorpio_ctrlset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/ScorpioPnC/CtrlSet")
install(FILES ${scorpio_taskset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/ScorpioPnC/TaskSet")
