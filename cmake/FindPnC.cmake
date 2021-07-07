# - Try to find PNC
#  Once done this will define
#  PNC_FOUND - System has Gurobi
#  PNC_INCLUDE_DIRS - The Gurobi include directories
#  PNC_LIBRARIES - The libraries needed to use Gurobi

find_path(PNC_INCLUDE_DIR
          NAMES configuration.hpp
          PATHS "/usr/local/include/PnC"
          )

find_library(DRACO_PNC_LIBRARY
             NAMES pnc-draco-pnc
             PATHS "/usr/local/lib"
             )

include(FindPackageHandleStandardArgs)

if(PNC_INCLUDE_DIR)
    set(PNC_INCLUDE_DIRS "${PNC_INCLUDE_DIR}" )
    set(PNC_LIBRARIES "${DRACO_PNC_LIBRARY}" )
    set(PNC_FOUND TRUE)
    message("-- Found PnC: TRUE")
else()
    message("-- Found PnC: FALSE, Build without PnC")
endif()

mark_as_advanced( PNC_INCLUDE_DIR
                  DRACO_PNC_LIBRARY )
