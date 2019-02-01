FILE(GLOB_RECURSE headers "*.h" "*.hpp")

install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myMIO")
