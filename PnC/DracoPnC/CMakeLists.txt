file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")
file(GLOB draco_testset_headers "TestSet/*.hpp")
file(GLOB draco_ctrlset_headers "CtrlSet/*.hpp")
file(GLOB draco_contactset_headers "ContactSet/*.hpp")

add_library(DracoPnC SHARED ${sources})
target_link_libraries(DracoPnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myMPC
                               myGaitCycle
                               myWBC
                               myFilters
                               myPlanner)

install(TARGETS DracoPnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC")
install(FILES ${draco_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/TestSet")
install(FILES ${draco_ctrlset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/CtrlSet")
install(FILES ${draco_contactset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/DracoPnC/ContactSet")
