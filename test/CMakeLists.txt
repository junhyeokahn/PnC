find_package(PkgConfig)
pkg_check_modules(GTEST "gtest" REQUIRED)

add_executable(qp_test qp_test.cpp)
target_link_libraries(qp_test pnc-goldfarb pnc-utils)

add_executable(draco_interface_test draco_interface_test.cpp)
target_link_libraries(draco_interface_test pnc-draco-pnc)

# tests using GoogleTest
add_executable(draco_state_estimator_test draco_kf_state_estimator_test.cpp)
target_link_libraries(draco_state_estimator_test ${GTEST_LIBRARIES} pnc-draco-pnc)

add_executable(dcm_planner_test dcm_planner_test.cpp)
target_link_libraries(dcm_planner_test ${GTEST_LIBRARIES} pnc-draco-pnc)