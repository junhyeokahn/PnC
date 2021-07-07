file(GLOB_RECURSE sources "*.cpp")
file(GLOB yaml_headers "include/yaml/*.h")
file(GLOB yaml_contrib_headers "include/yaml/contrib/*.h")
file(GLOB yaml_node_headers "include/yaml/node/*.h")
file(GLOB yaml_node_detail_headers "include/yaml/node/detail/*.h")

add_library(pnc-yaml ${sources}
                     ${yaml_headers}
                     ${yaml_contrib_headers}
                     ${yaml_node_headers}
                     ${yaml_node_detail_headers})

install(TARGETS pnc-yaml DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${yaml_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/yaml/include/yaml")
install(FILES ${yaml_contrib_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/yaml/include/yaml/contrib")
install(FILES ${yaml_node_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/yaml/include/yaml/node")
install(FILES ${yaml_node_detail_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/third_party/yaml/include/yaml/node/detail")
