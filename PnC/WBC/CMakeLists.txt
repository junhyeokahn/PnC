FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers "*.hpp")
FILE(GLOB ihwbc_headers "IHWBC/*.hpp")
FILE(GLOB manager_headers "Manager/*.hpp")

add_library(myWBC SHARED ${sources})

target_link_libraries(myWBC myUtils
                            myGoldfarb
                            myRobotSystem)

install(TARGETS myWBC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC")
install(FILES ${ihwbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC/IHWBC")
install(FILES ${manager_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC/Manager")
