file(GLOB sources "*.cpp")
file(GLOB planner_headers "*.hpp")
add_library(myPlanner SHARED ${sources})
target_link_libraries(myPlanner myUtils myRobotSystem)
install(TARGETS myPlanner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Planner")

## For TOWR+
#file(GLOB_RECURSE cpp_sources "*.cpp")
#file(GLOB_RECURSE cc_sources "*.cc")
#file(GLOB dcm_planner_headers "*.hpp")
#file(GLOB towr_plus_planner_headers "towr_plus/include/towr_plus/*.hpp")
#add_library(myPlanner SHARED ${cpp_sources} ${cc_sources})
#target_link_libraries(myPlanner myUtils myRobotSystem ifopt::ifopt_ipopt)
#install(TARGETS myPlanner DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${dcm_planner_headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/PnC/Planner")
#install(FILES ${towr_plus_planner_headers} DESTINATION
    #"${INSTALL_INCLUDE_DIR}/PnC/Planner/towr_plus/")
