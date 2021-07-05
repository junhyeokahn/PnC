FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB_RECURSE headers "*.hpp")

add_library(pnc-utils SHARED ${sources})
target_link_libraries(pnc-utils pnc-yaml)

install(TARGETS pnc-utils DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/utils")
