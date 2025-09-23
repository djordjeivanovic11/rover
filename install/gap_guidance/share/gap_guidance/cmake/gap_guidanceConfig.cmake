# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gap_guidance_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gap_guidance_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gap_guidance_FOUND FALSE)
  elseif(NOT gap_guidance_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gap_guidance_FOUND FALSE)
  endif()
  return()
endif()
set(_gap_guidance_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gap_guidance_FIND_QUIETLY)
  message(STATUS "Found gap_guidance: 0.1.0 (${gap_guidance_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gap_guidance' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gap_guidance_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gap_guidance_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gap_guidance_DIR}/${_extra}")
endforeach()
