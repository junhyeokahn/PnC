SET(GAIT_CYCLE_SOURCES GaitCycle.cpp)
SET(GAIT_CYCLE_HEADERS GaitCycle.hpp)

add_library(myGaitCycle SHARED ${GAIT_CYCLE_SOURCES})
target_link_libraries(myGaitCycle  myUtils)

add_executable(run_test_gait_cycle test_gait_cycle.cpp)
target_link_libraries(run_test_gait_cycle  myUtils
                                           myGaitCycle)

install(TARGETS myGaitCycle DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${GAIT_CYCLE_HEADERS} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/GaitCycle")
