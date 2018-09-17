FILE(GLOB interface_headers "include/solver/interface/*.h" "include/solver/interface/*.hpp")
FILE(GLOB optimizer_headers "include/solver/optimizer/*.h" "include/solver/optimizer/*.hpp")
file(GLOB_RECURSE sources "*.cpp")

add_library(myConicSolver SHARED ${sources} ${interface_headers}
    ${optimizer_headers})

target_link_libraries(myConicSolver myYaml)

install(TARGETS myConicSolver DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${interface_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/ConicSolver/include/solver/interface")
install(FILES ${optimizer_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/ConicSolver/include/solver/optimizer")
