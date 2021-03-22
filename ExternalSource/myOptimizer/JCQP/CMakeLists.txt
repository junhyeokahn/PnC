FILE(GLOB headers "*.hh" "*.h" "*.hpp" "amd/include/*.h")
file(GLOB_RECURSE sources "*.cpp" "amd/src/*.c" "amd/src/*.cpp")

add_library(myJCQP SHARED ${sources} ${headers})


#target_link_libraries(JCQP OsqpEigen::OsqpEigen osqp::osqp qpOASES pthread)
target_link_libraries(myJCQP pthread)

install(TARGETS myJCQP DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/JCQP")
