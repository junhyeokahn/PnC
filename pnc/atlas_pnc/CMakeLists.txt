file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(pnc-atlas-pnc ${sources})
target_link_libraries(pnc-atlas-pnc ${DART_LIBRARIES}
                                    pnc-robot-system
                                    pnc-utils
                                    pnc-filters
                                    pnc-dcm-planner
                                    pnc-wbc)
