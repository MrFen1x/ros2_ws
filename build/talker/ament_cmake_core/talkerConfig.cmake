# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_talker_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED talker_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(talker_FOUND FALSE)
  elseif(NOT talker_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(talker_FOUND FALSE)
  endif()
  return()
endif()
set(_talker_CONFIG_INCLUDED TRUE)

# output package information
if(NOT talker_FIND_QUIETLY)
  message(STATUS "Found talker: 0.0.0 (${talker_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'talker' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT talker_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(talker_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${talker_DIR}/${_extra}")
endforeach()
