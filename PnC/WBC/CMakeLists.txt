FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers "*.hpp")
FILE(GLOB wbdc_headers "WBDC/*.hpp")
FILE(GLOB wblc_headers "WBLC/*.hpp")
FILE(GLOB ihwbc_headers "IHWBC/*.hpp")

add_library(myWBC SHARED ${sources}
                         ${wbc_headers}
                         ${wblc_headers}
                         ${wbdc_headers}
                         ${ihwbc_headers})
target_link_libraries(myWBC myUtils
                            myGoldfarb
                            myRobotSystem)

install(TARGETS myWBC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC")
install(FILES ${wblc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC/WBLC")
install(FILES ${wbdc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC/WBDC")
install(FILES ${ihwbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC/WBDC")
