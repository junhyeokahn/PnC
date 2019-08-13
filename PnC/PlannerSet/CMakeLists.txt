file(GLOB_RECURSE sources "*.cpp")
file(GLOB centroid_planner_headers "CentroidPlanner/*.hpp")
file(GLOB pipm_foot_planner_headers "LIPMPlanner/*.hpp")
file(GLOB csg_headers "ContactSequenceGenerator/*.hpp")

add_library(myPlanner SHARED ${sources}
                             ${centroid_planner_headers}
                             ${pipm_foot_planner_headers}
                             ${csg_headers})

target_link_libraries(myPlanner myUtils
                                myConicSolver
                                myRobotSystem)

install(TARGETS myPlanner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${centroid_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/CentroidPlanner")
install(FILES ${pipm_foot_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/LIPMPlanner")
install(FILES ${csg_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/ContactSequenceGenerator")
