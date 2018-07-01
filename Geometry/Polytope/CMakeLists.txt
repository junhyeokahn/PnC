FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)

add_library (myPolytope SHARED ${header} ${source})

target_link_libraries (myPolytope myCdd
                                  ${MOSEK_LIBRARIES})
