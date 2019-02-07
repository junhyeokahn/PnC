FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE basic_headers "Basic/*.hpp")
FILE(GLOB_RECURSE kalman_headers "Kalman/*.hpp")

add_library(myFilters SHARED ${sources})
target_link_libraries(myFilters myUtils)

install(TARGETS myFilters DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${basic_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/Filters/Basic")
install(FILES ${kalman_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/Filters/Kalman")
