file(GLOB sources "*.cpp")
file(GLOB planner_headers "*.hpp")

add_library(myPlanner SHARED ${sources})

target_link_libraries(myPlanner myUtils myRobotSystem)

install(TARGETS myPlanner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Planner")
