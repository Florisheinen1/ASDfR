# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_filterer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED filterer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(filterer_FOUND FALSE)
  elseif(NOT filterer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(filterer_FOUND FALSE)
  endif()
  return()
endif()
set(_filterer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT filterer_FIND_QUIETLY)
  message(STATUS "Found filterer: 0.0.0 (${filterer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'filterer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT filterer_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(filterer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${filterer_DIR}/${_extra}")
endforeach()
