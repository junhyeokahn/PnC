file(GLOB srcs "*.cpp" "*.hpp")

add_executable(run_cart_pole ${srcs})
target_link_libraries(run_cart_pole ${DART_LIBRARIES}
                                    CartPolePnC
                                    myUtils)
