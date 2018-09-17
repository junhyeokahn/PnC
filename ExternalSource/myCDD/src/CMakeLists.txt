FILE(GLOB_RECURSE headers "*.h" "*.hpp")
file(GLOB_RECURSE sources *.c)

add_library (myCdd SHARED ${headers} ${sources})

install(TARGETS myCdd DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myCDD/src")
