# - Try to find GUROBI
#  Once done this will define
#  EIGEN_FOUND: TRUE iff Eigen is found.
#  EIGEN_INCLUDE_DIRS: Include directories for Eigen.
#  EIGEN_VERSION: Extracted from Eigen/src/Core/util/Macros.h

macro(EIGEN_REPORT_NOT_FOUND REASON_MSG)
  unset(EIGEN_FOUND)
  unset(EIGEN_INCLUDE_DIRS)
  unset(FOUND_INSTALLED_EIGEN_CMAKE_CONFIGURATION)
  # Make results of search visible in the CMake GUI if Eigen has not
  # been found so that user does not have to toggle to advanced view.
  mark_as_advanced(CLEAR EIGEN_INCLUDE_DIR)
  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
  # use the camelcase library name, not uppercase.
  if (Eigen_FIND_QUIETLY)
    message(STATUS "Failed to find Eigen - " ${REASON_MSG} ${ARGN})
  elseif (Eigen_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find Eigen - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use no priority which emits a message
    # but continues configuration and allows generation.
    message("-- Failed to find Eigen - " ${REASON_MSG} ${ARGN})
  endif ()
  return()
endmacro(EIGEN_REPORT_NOT_FOUND)

# Protect against any alternative find_package scripts for this library having
# been called previously (in a client project) which set EIGEN_FOUND, but not
# the other variables we require / set here which could cause the search logic
# here to fail.
unset(EIGEN_FOUND)

# -----------------------------------------------------------------
# By default, if the user has expressed no preference for using an exported
# Eigen CMake configuration over performing a search for the installed
# components, and has not specified any hints for the search locations, then
# prefer an exported configuration if available.
if (NOT DEFINED EIGEN_PREFER_EXPORTED_EIGEN_CMAKE_CONFIGURATION
    AND NOT EIGEN_INCLUDE_DIR_HINTS)
  message(STATUS "No preference for use of exported Eigen CMake configuration "
    "set, and no hints for include directory provided. "
    "Defaulting to preferring an installed/exported Eigen CMake configuration "
    "if available.")
  set(EIGEN_PREFER_EXPORTED_EIGEN_CMAKE_CONFIGURATION TRUE)
endif()

if (EIGEN_PREFER_EXPORTED_EIGEN_CMAKE_CONFIGURATION)
  # Try to find an exported CMake configuration for Eigen.
  #
  # We search twice, s/t we can invert the ordering of precedence used by
  # find_package() for exported package build directories, and installed
  # packages (found via CMAKE_SYSTEM_PREFIX_PATH), listed as items 6) and 7)
  # respectively in [1].
  #
  # By default, exported build directories are (in theory) detected first, and
  # this is usually the case on Windows.  However, on OS X & Linux, the install
  # path (/usr/local) is typically present in the PATH environment variable
  # which is checked in item 4) in [1] (i.e. before both of the above, unless
  # NO_SYSTEM_ENVIRONMENT_PATH is passed).  As such on those OSs installed
  # packages are usually detected in preference to exported package build
  # directories.
  #
  # To ensure a more consistent response across all OSs, and as users usually
  # want to prefer an installed version of a package over a locally built one
  # where both exist (esp. as the exported build directory might be removed
  # after installation), we first search with NO_CMAKE_PACKAGE_REGISTRY which
  # means any build directories exported by the user are ignored, and thus
  # installed directories are preferred.  If this fails to find the package
  # we then research again, but without NO_CMAKE_PACKAGE_REGISTRY, so any
  # exported build directories will now be detected.
  #
  # To prevent confusion on Windows, we also pass NO_CMAKE_BUILDS_PATH (which
  # is item 5) in [1]), to not preferentially use projects that were built
  # recently with the CMake GUI to ensure that we always prefer an installed
  # version if available.
  #
  # [1] http://www.cmake.org/cmake/help/v2.8.11/cmake.html#command:find_package
  find_package(Eigen3 QUIET
                      NO_MODULE
                      NO_CMAKE_PACKAGE_REGISTRY
                      NO_CMAKE_BUILDS_PATH)
  if (EIGEN3_FOUND)
    message(STATUS "Found installed version of Eigen: ${Eigen3_DIR}")
  else()
    # Failed to find an installed version of Eigen, repeat search allowing
    # exported build directories.
    message(STATUS "Failed to find installed Eigen CMake configuration, "
      "searching for Eigen build directories exported with CMake.")
    # Again pass NO_CMAKE_BUILDS_PATH, as we know that Eigen is exported and
    # do not want to treat projects built with the CMake GUI preferentially.
    find_package(Eigen3 QUIET
                        NO_MODULE
                        NO_CMAKE_BUILDS_PATH)
    if (EIGEN3_FOUND)
      message(STATUS "Found exported Eigen build directory: ${Eigen3_DIR}")
    endif()
  endif()
  if (EIGEN3_FOUND)
    set(FOUND_INSTALLED_EIGEN_CMAKE_CONFIGURATION TRUE)
    set(EIGEN_FOUND ${EIGEN3_FOUND})
    set(EIGEN_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}" CACHE STRING
      "Eigen include directory" FORCE)
  else()
    message(STATUS "Failed to find an installed/exported CMake configuration "
      "for Eigen, will perform search for installed Eigen components.")
  endif()
endif()

if (NOT EIGEN_FOUND)
  # Search user-installed locations first, so that we prefer user installs
  # to system installs where both exist.
  list(APPEND EIGEN_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
  # Additional suffixes to try appending to each search path.
  list(APPEND EIGEN_CHECK_PATH_SUFFIXES
    eigen3 # Default root directory for Eigen.
    Eigen/include/eigen3 # Windows (for C:/Program Files prefix) < 3.3
    Eigen3/include/eigen3 ) # Windows (for C:/Program Files prefix) >= 3.3

  # Search supplied hint directories first if supplied.
  find_path(EIGEN_INCLUDE_DIR
    NAMES Eigen/Core
    HINTS ${EIGEN_INCLUDE_DIR_HINTS}
    PATHS ${EIGEN_CHECK_INCLUDE_DIRS}
    PATH_SUFFIXES ${EIGEN_CHECK_PATH_SUFFIXES})

  if (NOT EIGEN_INCLUDE_DIR OR
      NOT EXISTS ${EIGEN_INCLUDE_DIR})
    eigen_report_not_found(
      "Could not find eigen3 include directory, set EIGEN_INCLUDE_DIR to "
      "path to eigen3 include directory, e.g. /usr/local/include/eigen3.")
  endif (NOT EIGEN_INCLUDE_DIR OR
    NOT EXISTS ${EIGEN_INCLUDE_DIR})

  # Mark internally as found, then verify. EIGEN_REPORT_NOT_FOUND() unsets
  # if called.
  set(EIGEN_FOUND TRUE)
endif()

# Extract Eigen version from Eigen/src/Core/util/Macros.h
if (EIGEN_INCLUDE_DIR)
  set(EIGEN_VERSION_FILE ${EIGEN_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h)
  if (NOT EXISTS ${EIGEN_VERSION_FILE})
    eigen_report_not_found(
      "Could not find file: ${EIGEN_VERSION_FILE} "
      "containing version information in Eigen install located at: "
      "${EIGEN_INCLUDE_DIR}.")
  else (NOT EXISTS ${EIGEN_VERSION_FILE})
    file(READ ${EIGEN_VERSION_FILE} EIGEN_VERSION_FILE_CONTENTS)

    string(REGEX MATCH "#define EIGEN_WORLD_VERSION [0-9]+"
      EIGEN_WORLD_VERSION "${EIGEN_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define EIGEN_WORLD_VERSION ([0-9]+)" "\\1"
      EIGEN_WORLD_VERSION "${EIGEN_WORLD_VERSION}")

    string(REGEX MATCH "#define EIGEN_MAJOR_VERSION [0-9]+"
      EIGEN_MAJOR_VERSION "${EIGEN_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define EIGEN_MAJOR_VERSION ([0-9]+)" "\\1"
      EIGEN_MAJOR_VERSION "${EIGEN_MAJOR_VERSION}")

    string(REGEX MATCH "#define EIGEN_MINOR_VERSION [0-9]+"
      EIGEN_MINOR_VERSION "${EIGEN_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define EIGEN_MINOR_VERSION ([0-9]+)" "\\1"
      EIGEN_MINOR_VERSION "${EIGEN_MINOR_VERSION}")

    # This is on a single line s/t CMake does not interpret it as a list of
    # elements and insert ';' separators which would result in 3.;2.;0 nonsense.
    set(EIGEN_VERSION "${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}")
  endif (NOT EXISTS ${EIGEN_VERSION_FILE})
endif (EIGEN_INCLUDE_DIR)

# Set standard CMake FindPackage variables if found.
if (EIGEN_FOUND)
  set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})
endif (EIGEN_FOUND)

# Handle REQUIRED / QUIET optional arguments and version.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen
  REQUIRED_VARS EIGEN_INCLUDE_DIRS
  VERSION_VAR EIGEN_VERSION)

# Only mark internal variables as advanced if we found Eigen, otherwise
# leave it visible in the standard GUI for the user to set manually.
if (EIGEN_FOUND)
  mark_as_advanced(FORCE EIGEN_INCLUDE_DIR
    Eigen3_DIR) # Autogenerated by find_package(Eigen3)
endif (EIGEN_FOUND)
