pybind11_add_module(atlas-interface atlas_interface_pybind.cpp)
target_link_libraries(atlas-interface PRIVATE AtlasPnC)

