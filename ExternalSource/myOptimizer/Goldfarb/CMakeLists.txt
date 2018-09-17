FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)
add_library (myGoldfarb SHARED ${headers} ${sources})

install(TARGETS myGoldfarb DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/Goldfarb")
