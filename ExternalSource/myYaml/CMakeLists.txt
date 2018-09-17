file(GLOB_RECURSE sources "*.cpp")
file(GLOB yaml_headers "include/myYaml/*.h")
file(GLOB yaml_contrib_headers "include/myYaml/contrib/*.h")
file(GLOB yaml_node_headers "include/myYaml/node/*.h")
file(GLOB yaml_node_detail_headers "include/myYaml/node/detail/*.h")

add_library(myYaml SHARED ${sources}
                          ${yaml_headers}
                          ${yaml_contrib_headers}
                          ${yaml_node_headers}
                          ${yaml_node_detail_headers})

install(TARGETS myYaml DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${yaml_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myYaml/include/myYaml")
install(FILES ${yaml_contrib_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myYaml/include/myYaml/contrib")
install(FILES ${yaml_node_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myYaml/include/myYaml/node")
install(FILES ${yaml_node_detail_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/ExternalSource/myYaml/include/myYaml/node/detail")
