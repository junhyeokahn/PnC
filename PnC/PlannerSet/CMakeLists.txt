file(GLOB_RECURSE srcs "*.cpp" "*.hpp")

file(GLOB_RECURSE sources "*.cpp")
file(GLOB centroid_planner_headers "CentroidPlanner/*.hpp")
file(GLOB inv_kin_planner_headers "InvKinPlanner/*.hpp")

add_library(myPlanner SHARED ${sources} ${centroid_planner_headers}
                             ${inv_kin_planner_headers})

target_link_libraries(myPlanner myUtils
                                myContact
                                myConicSolver
                                myRobotSystem)

install(TARGETS myPlanner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${centroid_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/CentroidPlanner")
install(FILES ${inv_kin_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/InvKinPlanner")
