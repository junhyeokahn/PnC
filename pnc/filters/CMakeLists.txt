FILE(GLOB sources "*.cpp")
FILE(GLOB headers "*.hpp")

add_library(pnc-filters SHARED ${sources})
target_link_libraries(pnc-filters pnc-utils)

install(TARGETS pnc-filters DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/filters")
