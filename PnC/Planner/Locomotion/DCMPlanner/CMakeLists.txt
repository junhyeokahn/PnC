FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(myDCMPlanner SHARED ${sources})
target_link_libraries(myDCMPlanner myUtils)

install(TARGETS myDCMPlanner DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Planner/Locomotion/DCMPlanner")
