add_library(myPolytope SHARED Polytope.cpp Polytope.h)
target_link_libraries(myPolytope myCdd)

if(MOSEK_FOUND)
    add_library(myLownerJohnEllipsoid LownerJohnEllipsoid.cpp
                                      LownerJohnEllipsoid.hpp
                                      ${MOSEK_LIBRARIES})
    target_link_libraries(myPolytope myLownerJohnEllipsoid)
endif()
