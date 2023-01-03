FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB_RECURSE headers "*.hpp")
FILE(GLOB interpolation_sources "interpolation/*.cpp")
FILE(GLOB interpolation_headers "interpolation/*.hpp")

add_library(pnc-utils ${sources})
target_link_libraries(pnc-utils pnc-yaml)

install(TARGETS pnc-utils DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/utils")
