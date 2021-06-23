FILE(GLOB headers "*.h")
FILE(GLOB_RECURSE sources *.c)

set(IS_LINUX ON)


add_library (myOSQP SHARED ${headers} ${sources})

install(TARGETS myOSQP DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/osqp")
