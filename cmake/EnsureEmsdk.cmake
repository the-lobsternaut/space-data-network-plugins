# EnsureEmsdk.cmake — Auto-install emsdk if not present
#
# Usage in plugin CMakeLists.txt:
#   include(${CMAKE_SOURCE_DIR}/../../cmake/EnsureEmsdk.cmake)
#   # or copy this file into your plugin's cmake/ directory
#
# This module checks for deps/emsdk/ and installs it if missing.
# It's meant to be run BEFORE the emcmake/emmake invocation,
# typically from a build.sh wrapper script.

# The actual auto-install happens in build.sh since CMake itself
# can't bootstrap the toolchain it's being invoked with.
# This file provides the emsdk path variables for CMake.

# Find emsdk relative to the plugin root
set(PLUGIN_ROOT "${CMAKE_SOURCE_DIR}/../..")
get_filename_component(PLUGIN_ROOT "${PLUGIN_ROOT}" ABSOLUTE)

set(EMSDK_DIR "${PLUGIN_ROOT}/deps/emsdk")

if(NOT EXISTS "${EMSDK_DIR}/upstream/emscripten")
    message(WARNING "emsdk not found at ${EMSDK_DIR}. Run build.sh first to auto-install.")
endif()
