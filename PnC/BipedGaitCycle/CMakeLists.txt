SET(BIPED_GAIT_CYCLE_SOURCES BipedGaitCycle.cpp)
SET(BIPED_GAIT_CYCLE_HEADERS BipedGaitCycle.hpp)

add_library(myBipedGaitCycle SHARED ${BIPED_GAIT_CYCLE_SOURCES})
target_link_libraries(myBipedGaitCycle  myUtils)

add_executable(run_test_gait_cycle test_gait_cycle.cpp)
target_link_libraries(run_test_gait_cycle  myUtils
										   myBipedGaitCycle)

install(TARGETS myBipedGaitCycle DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${BIPED_GAIT_CYCLE_HEADERS} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/BipedGaitCycle")
