# - Try to find GUROBI
#  Once done this will define
#  GUROBI_FOUND - System has Gurobi
#  GUROBI_INCLUDE_DIRS - The Gurobi include directories
#  GUROBI_LIBRARIES - The libraries needed to use Gurobi

set (GUROBI_HOME $ENV{GUROBI_HOME})

find_path(GUROBI_INCLUDE_DIR
          NAMES gurobi_c++.h
          PATHS "${GUROBI_HOME}/include"
          )

find_library(GUROBI_LIBRARY
             NAMES gurobi
                   gurobi45
                   gurobi46
                   gurobi50
                   gurobi51
                   gurobi52
                   gurobi55
                   gurobi56
                   gurobi60
                   gurobi65
                   gurobi70
                   gurobi80
              PATHS "${GUROBI_HOME}/lib"
              )

find_library(GUROBI_CXX_LIBRARY
             NAMES gurobi_c++
             PATHS "$ENV{GUROBI_HOME}/lib"
             )

include(FindPackageHandleStandardArgs)

if(GUROBI_INCLUDE_DIR)
    set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIR}" )
    set(GUROBI_LIBRARIES "${GUROBI_LIBRARY};${GUROBI_CXX_LIBRARY}" )
    set(GUROBI_FOUND TRUE)
    message("-- Found Gurobi: TRUE")
else()
    message("-- Found Gurobi: FALSE, Build without Gurobi")
endif()

mark_as_advanced(GUROBI_HOME
                 GUROBI_INCLUDE_DIR
                 GUROBI_LIBRARY
                 GUROBI_CXX_LIBRARY
                 )
