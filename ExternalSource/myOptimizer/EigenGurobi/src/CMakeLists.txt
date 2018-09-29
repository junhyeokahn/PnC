set(sources Gurobi.cpp)
set(headers eigen_gurobi_api.h Gurobi.h)

add_library(myEigenGurobi SHARED ${sources} ${headers})
target_link_libraries(myEigenGurobi ${GUROBI_LIBRARIES})

install(TARGETS myEigenGurobi DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/EigenGurobi/src")
