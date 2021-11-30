file(GLOB sources "*.cpp")
file(GLOB csg_headers "*.hpp")

add_library(pnc-csg SHARED ${sources}
                           ${csg_headers})
#target_link_libraries(pnc_csg pnc_centroid_planner)

install(TARGETS pnc-csg DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${csg_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/planners/locomotion/contact_sequence_generator")



