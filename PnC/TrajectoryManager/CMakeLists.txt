file(GLOB sources "*.cpp")
file(GLOB tm_headers "*.hpp")

add_library(myTrajectoryManager SHARED ${sources})

target_link_libraries(myTrajectoryManager myUtils myRobotSystem)

install(TARGETS myTrajectoryManager DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${tm_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/TrajectoryManager")
