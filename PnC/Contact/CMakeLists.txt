file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(myContact SHARED ${sources})
target_link_libraries(myContact ${DART_LIBRARIES}
                                myPolytope)

install(TARGETS myContact DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/Contact")
