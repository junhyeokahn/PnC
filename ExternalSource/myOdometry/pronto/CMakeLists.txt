FILE(GLOB headers "include/*.h" "include/*.hpp")
FILE(GLOB_RECURSE sources src/*.cpp)
include_directories(/home/knapsack/project/locomotion/src/PnC_june/ExternalSource/myOdometry/pronto/include)

add_library (myOdometry SHARED ${headers} ${sources})

install(TARGETS myOdometry DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myOdometry/pronto/include")
