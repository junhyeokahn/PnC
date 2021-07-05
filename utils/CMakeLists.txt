FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB_RECURSE headers "*.hpp")

add_library(pnc-util SHARED ${sources})
target_link_libraries(pnc-util pnc-yaml)

install(TARGETS pnc-util DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/utils")
