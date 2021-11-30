file(GLOB sources "*.cpp")
file(GLOB centroid_planner_headers "*.hpp")

add_library(pnc_centroid_planner SHARED ${sources}
                                        ${centroid_planner_headers})
target_link_libraries(pnc_centroid_planner pnc-utils
                                           pnc-robot-system
                                           pnc-csg
                                           pnc-yaml
                                           myConicSolver)

install(TARGETS pnc_centroid_planner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${centroid_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/planners/locomotion/centroid_planner")



