file(GLOB_RECURSE sources "*.cpp" "*.c")
file(GLOB robot_system_headers "*.hpp")
file(GLOB draco_kin_headers
    "RobotModel/Robot/Draco/DracoGen/kin_eigen/include/*.h")
file(GLOB draco_dyn_headers
    "RobotModel/Robot/Draco/DracoGen/dyn_eigen/include/*.h")

add_library(myRobotSystem SHARED ${sources}
                                 ${robot_system_headers}
                                 ${draco_dyn_headers}
                                 ${draco_kin_headers})

target_link_libraries(myRobotSystem ${DART_LIBRARIES})

install(TARGETS myRobotSystem DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${robot_system_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/RobotSystem")
install(FILES ${draco_kin_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include")
install(FILES ${draco_dyn_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/RobotSystem/RobotModel/Robot/Draco/DracoGen/dyn_eigen/include")
