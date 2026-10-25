# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fullrobotassempackage_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fullrobotassempackage_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fullrobotassempackage_FOUND FALSE)
  elseif(NOT fullrobotassempackage_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fullrobotassempackage_FOUND FALSE)
  endif()
  return()
endif()
set(_fullrobotassempackage_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fullrobotassempackage_FIND_QUIETLY)
  message(STATUS "Found fullrobotassempackage: 1.0.0 (${fullrobotassempackage_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fullrobotassempackage' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT fullrobotassempackage_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fullrobotassempackage_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fullrobotassempackage_DIR}/${_extra}")
endforeach()
