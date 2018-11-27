file(GLOB sources "*.cpp" "*.c")
file(GLOB robot_system_headers "*.hpp")

add_library(myRobotSystem SHARED ${sources}
                                 ${robot_system_headers})

target_link_libraries(myRobotSystem ${DART_LIBRARIES}
                                    myUtils)

install(TARGETS myRobotSystem DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${robot_system_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/RobotSystem")
