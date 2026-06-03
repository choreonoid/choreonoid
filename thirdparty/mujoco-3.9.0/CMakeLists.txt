# MuJoCo 3.9.0 bundled for Choreonoid
#
# This directory ships the official prebuilt MuJoCo dynamic library for the
# platforms Choreonoid supports, so the MuJoCoPlugin can be built without any
# external MuJoCo installation. MuJoCo exposes a pure C API and links its C++
# dependencies (libstdc++, tinyxml2, qhull, ...) statically with hidden symbol
# visibility, so it does not clash with Choreonoid's own runtime.
#
# Provided:
#     Linux   x86_64 : lib/linux-x86_64/libmujoco.so.3.9.0
#     Windows x86_64 : lib/windows-x86_64/mujoco.dll + mujoco.lib
# Headers (shared by all platforms): include/mujoco/
#
# This file defines the IMPORTED target "mujoco" that plugins link against,
# copies the runtime library into the build tree, and installs it.
#
# See CHOREONOID_README.md for the procedure used to assemble this directory
# when updating MuJoCo.

cmake_minimum_required(VERSION 3.16)

set(MUJOCO_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")

add_library(mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)
# Treat the MuJoCo headers as system headers so plugins do not inherit any
# warnings from them.
set_target_properties(mujoco PROPERTIES
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

if(UNIX AND NOT APPLE)
  set(_mujoco_soname "libmujoco.so.3.9.0")
  set(_mujoco_lib "${MUJOCO_ROOT_DIR}/lib/linux-x86_64/${_mujoco_soname}")

  # Place the shared library where the build tree's RPATH can find it
  # (CMAKE_LIBRARY_OUTPUT_DIRECTORY is on the plugin RPATH), and link against
  # that copy. The SONAME recorded in DT_NEEDED is "libmujoco.so.3.9.0".
  file(COPY "${_mujoco_lib}" DESTINATION "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

  set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${_mujoco_soname}"
  )

  install(FILES "${_mujoco_lib}" DESTINATION ${CHOREONOID_LIB_SUBDIR})

elseif(WIN32)
  set(_mujoco_dll "${MUJOCO_ROOT_DIR}/lib/windows-x86_64/mujoco.dll")
  set(_mujoco_implib "${MUJOCO_ROOT_DIR}/lib/windows-x86_64/mujoco.lib")

  # Copy the DLL next to the executables so it is found at runtime.
  file(COPY "${_mujoco_dll}" DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

  set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION "${_mujoco_dll}"
    IMPORTED_IMPLIB "${_mujoco_implib}"
  )

  install(FILES "${_mujoco_dll}" DESTINATION ${CHOREONOID_BIN_SUBDIR})

else()
  message(FATAL_ERROR
    "Bundled MuJoCo supports Linux (x86_64) and Windows (x86_64) only. "
    "Set ENABLE_MUJOCO=OFF to skip it.")
endif()
