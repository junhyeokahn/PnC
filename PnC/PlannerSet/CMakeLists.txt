file(GLOB_RECURSE sources "*.cpp")
file(GLOB centroid_planner_headers "CentroidPlanner/*.hpp")
file(GLOB contact_sequence_headers "ContactSequenceGenerator/*.hpp")
file(GLOB pipm_foot_planner_headers "PIPM_FootPlacementPlanner/*.hpp")
file(GLOB dcm_planner_headers "DCMPlanner/*.hpp")

if(NOT USE_DRAKE)
    list(REMOVE_ITEM sources
        ${CMAKE_CURRENT_SOURCE_DIR}/CentroidPlanner/PrePlannedCentroidPlanner.cpp)
    list(REMOVE_ITEM centroid_planner_headers
        ${CMAKE_CURRENT_SOURCE_DIR}/CentroidPlanner/PrePlannedCentroidPlanner.hpp)
endif()

add_library(myPlanner SHARED ${sources}
                             ${centroid_planner_headers}
                             ${pipm_foot_planner_headers}
                             ${contact_sequence_headers}
                             ${dcm_planner_headers})

if(USE_DRAKE)
  target_link_libraries(myPlanner drake::drake)
endif()

target_link_libraries(myPlanner myUtils
                                myConicSolver
                                myRobotSystem)

install(TARGETS myPlanner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${centroid_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/CentroidPlanner")
install(FILES ${pipm_foot_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/PIPM_FootPlacementPlanner")
install(FILES ${contact_sequence_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/ContactSequenceGenerator")
install(FILES ${dcm_planner_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/PlannerSet/DCMPlanner")
