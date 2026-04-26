option(BUILD_PHYSX_PLUGIN "Building PhysXPlugin" ${ENABLE_PHYSX})
if(NOT BUILD_PHYSX_PLUGIN)
  return()
endif()

set(PHYSX_LIBRARIES
  # The order of the PhysX static libraries matters for GCC/Clang linkers.
  # Libraries that depend on others must be listed first (dependee before dependency).
  # Changing the order may cause undefined symbol errors on Linux.
  PhysXExtensions_static_64
  PhysXCooking_static_64
  PhysX_static_64
  PhysXPvdSDK_static_64
  PhysXCommon_static_64
  PhysXFoundation_static_64
  )

if(NOT ENABLE_PHYSX)
  # Using an externally built PhysX. The user must set PHYSX_DIR to the
  # top-level directory of the pre-built PhysX SDK. Enable ENABLE_PHYSX if
  # you want to build the bundled PhysX instead.
  set(PHYSX_DIR ${PHYSX_DIR} CACHE PATH "Set the top directory of PhysX 5")

  if(NOT PHYSX_DIR)
    message(FATAL_ERROR "Please specify the directory of PhysX 5 to PHYSX_DIR, or enable ENABLE_PHYSX.")
    return()
  endif()

  set(PHYSX_BINARY_TYPE release CACHE STRING "Specify the binary type among release, profile, checked, debug")

  include_directories(${PHYSX_DIR}/include)
  link_directories(${PHYSX_DIR}/bin/linux.x86_64/${PHYSX_BINARY_TYPE})
endif()

set(target CnoidPhysXPlugin)

set(sources
  PhysXPlugin.cpp
  PhysXSimulatorItem.cpp
  PxContinuousTrack.cpp
  PxContinuousTrackSimulator.cpp
  )

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "PX_PHYSX_STATIC_LIB")
target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE ${PHYSX_LIBRARIES})

# Suppress LTO-time warnings originating from the bundled PhysX sources.
#
# When Choreonoid is built with ENABLE_LTO=ON, GCC performs whole-program
# analysis at link time and re-emits warnings for inlined PhysX code. The
# per-source -Wno-* flags set in thirdparty/physx-5.6.1/CMakeLists.txt
# (PHYSX_GCC_WARNING_FLAGS) do not apply to this link-time pass, so the
# noise has to be silenced on the final link step, which happens when this
# target (CnoidPhysXPlugin) is linked.
#
# -Wodr              : PhysX declares several classes/structs with the same
#                      name in multiple translation units (Scene, NpScene,
#                      SolverDt, SortedTriangle). The layouts are effectively
#                      identical but LTO flags them as ODR violations.
# -Wstringop-overread: LTO inlining makes some PhysX memcpy sites look like
#                      they read past the source object (for example the
#                      PxMat33 access in DyFeatherstoneForwardDynamic.cpp).
#                      The paths are not reached at runtime.
#
# Both are upstream-code issues we do not want to patch; they are already
# suppressed at compile time for the PhysX targets themselves. The link-time
# re-emission only happens under LTO, so the suppression is gated on
# ENABLE_LTO to avoid masking any unrelated future warnings in non-LTO builds.
if(ENABLE_LTO AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_options(${target} PRIVATE -Wno-odr -Wno-stringop-overread)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  target_compile_definitions(${target} PUBLIC _DEBUG)
endif()
