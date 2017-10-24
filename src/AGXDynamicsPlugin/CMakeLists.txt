option(BUILD_AGX_DYNAMICS_PLUGIN "Building AGXDynamicsPlugin" OFF)
option(AGX_USE_AGGRESSIVE_WARNINGS "Enable to activate aggressive warnings" OFF)

if(NOT BUILD_AGX_DYNAMICS_PLUGIN)
  return()
endif()

if("$ENV{AGX_DIR}"  STREQUAL "")
  MESSAGE(WARNING "\n*** The environment initialization file setup_env.bat/bash has not been executed\n*** Errors might occur during runtime!")
endif()

if(NOT EXISTS ${AGX_DIR})
  set(AGX_DIR $ENV{AGX_DIR} CACHE PATH "set the top directory of the AGX Dynamics library" FORCE)
endif()

set(CMAKE_MODULE_PATH "${AGX_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
include(AGXUtils)
include(AGX${CMAKE_SYSTEM_NAME}Specific)

include(AGXCompilerFlags )

# Now locate AgX
find_package(AGX REQUIRED)

if(NOT AGX_FOUND)
  MESSAGE(FATAL_ERROR "Unable to locate AGX headers and libraries. Cannot proceed.")
endif()


#add_definitions(${AGX_CFLAGS})
include_directories(${AGX_INCLUDE_DIR})
link_directories(${AGX_LIBRARY_DIR})

set(target CnoidAGXDynamicsPlugin)

set(sources
  AGXBody.cpp
  AGXObjectFactory.cpp
  AGXScene.cpp
  AGXDynamicsPlugin.cpp
  AGXSimulatorItem.cpp
  AGXSimulatorItemImpl.cpp
  AGXVehicleContinuousTrack.cpp
  AGXVehicleContinuousTrackDevice.cpp
  AGXBodyExtension.cpp
  AGXConvexDecomposition.cpp
  )

set(headers
  AGXBody.h
  AGXInclude.h
  AGXObjectFactory.h
  AGXScene.h
  AGXSimulatorItem.h
  AGXSimulatorItemImpl.h
  AGXVehicleContinuousTrack.h
  AGXVehicleContinuousTrackDevice.h
  AGXBodyExtension.h
  AGXConvexDecomposition.h
  AGXConvert.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})

# Specify which libraries to be used when building in release
AGX_TARGET_LINK_LIBRARIES(${target} optimized ${AGX_LIBRARIES_RELEASE})

# Specify which libraries to be used when building in debug
if(AGX_LIBRARIES_DEBUG)
  AGX_TARGET_LINK_LIBRARIES(${target} debug ${AGX_LIBRARIES_DEBUG} )
else()
  AGX_TARGET_LINK_LIBRARIES(${target} debug ${AGX_LIBRARIES_RELEASE} )
endif()

target_link_libraries(${target} CnoidBodyPlugin)

get_target_property(compile_flags ${target} COMPILE_FLAGS)
if(NOT compile_flags)
  set(compile_flags "")
endif()
set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${compile_flags} -std=c++11 -msse3")
apply_common_setting_for_plugin(${target} "${headers}")

add_subdirectory(AGXSubSimulator)
add_subdirectory(AGXBodyExtensionPlugin)
