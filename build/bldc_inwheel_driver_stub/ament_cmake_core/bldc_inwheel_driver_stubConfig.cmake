# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bldc_inwheel_driver_stub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bldc_inwheel_driver_stub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bldc_inwheel_driver_stub_FOUND FALSE)
  elseif(NOT bldc_inwheel_driver_stub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bldc_inwheel_driver_stub_FOUND FALSE)
  endif()
  return()
endif()
set(_bldc_inwheel_driver_stub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bldc_inwheel_driver_stub_FIND_QUIETLY)
  message(STATUS "Found bldc_inwheel_driver_stub: 0.0.0 (${bldc_inwheel_driver_stub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bldc_inwheel_driver_stub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bldc_inwheel_driver_stub_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bldc_inwheel_driver_stub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bldc_inwheel_driver_stub_DIR}/${_extra}")
endforeach()
