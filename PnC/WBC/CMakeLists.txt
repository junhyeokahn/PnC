FILE(GLOB sources *.cpp)
FILE(GLOB wbc_headers "*.hpp")
FILE(GLOB wblc_sources "WBLC/*.cpp")

add_library(myWBC SHARED ${sources}
                         ${wbc_headers}
                         ${wblc_sources})

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