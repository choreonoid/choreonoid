option(BUILD_PHYSX_PLUGIN "Building PhysXPlugin" OFF)
if(NOT BUILD_PHYSX_PLUGIN)
  return()
endif()

set(PHYSX_DIR ${PHYSX_DIR} CACHE PATH "Set the top directory of PhysX 5")

if(NOT PHYSX_DIR)
  message(FATAL_ERROR "Please specify the directory of PhysX 5 to PHYSX_DIR.")
  return()
endif()

set(PHYSX_BINARY_TYPE release CACHE STRING "Specify the binary type among release, profile, checked, debug")

set(PHYSX_INCLUDE_DIRS ${PHYSX_DIR}/include)
set(PHYSX_LIBRARY_DIRS ${PHYSX_DIR}/bin/linux.x86_64/${PHYSX_BINARY_TYPE})
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

include_directories(${PHYSX_INCLUDE_DIRS})
link_directories(${PHYSX_LIBRARY_DIRS})

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

if(CMAKE_BUILD_TYPE MATCHES Debug)
  target_compile_definitions(${target} PUBLIC _DEBUG)
endif()
