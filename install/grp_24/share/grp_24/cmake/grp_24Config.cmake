# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_grp_24_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED grp_24_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(grp_24_FOUND FALSE)
  elseif(NOT grp_24_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(grp_24_FOUND FALSE)
  endif()
  return()
endif()
set(_grp_24_CONFIG_INCLUDED TRUE)

# output package information
if(NOT grp_24_FIND_QUIETLY)
  message(STATUS "Found grp_24: 0.0.0 (${grp_24_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'grp_24' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT grp_24_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(grp_24_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${grp_24_DIR}/${_extra}")
endforeach()
