FILE(GLOB sources *.cpp)
FILE(GLOB wbc_headers "*.hpp")

add_library(myWBC SHARED ${sources})

target_link_libraries(myWBC myUtils
                            myGoldfarb
                            myRobotSystem)

install(TARGETS myWBC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/WBC")
