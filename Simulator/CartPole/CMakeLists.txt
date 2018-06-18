file(GLOB srcs "*.cpp" "*.hpp")

add_executable(cart_pole ${srcs})
target_link_libraries(cart_pole ${DART_LIBRARIES}
                                CartPolePnC)
