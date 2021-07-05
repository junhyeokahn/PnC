FILE(GLOB headers "*.hpp")

install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/kalman_filters")
