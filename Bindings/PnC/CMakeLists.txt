pybind11_add_module(DracoInterface DracoInterfacePy.cpp)
target_link_libraries(DracoInterface PRIVATE DracoPnC)
<<<<<<< HEAD
set_target_properties(DracoPnC PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

pybind11_add_module(FixedAtlasInterface FixedAtlasInterfacePy.cpp)
target_link_libraries(FixedAtlasInterface PRIVATE FixedAtlasPnC)
set_target_properties(FixedAtlasPnC PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
=======
set_target_properties(DracoInterface PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
>>>>>>> cdb8ed94b069484c84da0e22aa7d4eb6ebe213ef
