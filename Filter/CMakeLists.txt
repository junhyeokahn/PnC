FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library(myFilter SHARED ${sources} ${headers})
target_link_libraries(myFilter myUtils)

install(TARGETS myFilter DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Filter")
