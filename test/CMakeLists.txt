add_executable(qp_test qp_test.cpp)
target_link_libraries(qp_test pnc-goldfarb pnc-utils)

add_executable(draco_interface_test draco_interface_test.cpp)
target_link_libraries(draco_interface_test pnc-draco-pnc)
