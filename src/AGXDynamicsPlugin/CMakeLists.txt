option(BUILD_AGX_DYNAMICS_PLUGIN "Building AGXDynamicsPlugin" OFF)
option(AGX_USE_AGGRESSIVE_WARNINGS "Enable to activate aggressive warnings" OFF)

if(NOT BUILD_AGX_DYNAMICS_PLUGIN)
  return()
endif()

if(UNIX AND NOT DEFINED ENV{AGX_DIR})
  message(WARNING "\n*** AGXDynamicsPlugin: The environment initialization file setup_env.bat/bash has not been executed\n*** Errors might occur during runtime!")
endif()

if(WIN32)
  option(INSTALL_AGX_RUNTIME "Install the runtime libraries of AGX Dynamics" ON)
endif()

# Check for architecture
set(AGX_BUILD_ARCHITECTURE "")
if(CMAKE_SIZEOF_VOID_P MATCHES "8")
   set(AGX_64BIT_ARCHITECTURE 1)
else()
endif()

if(MSVC)
  set(AGX_DEBUG_POSTFIX "d")
  set(CMAKE_DEBUG_POSTFIX ${AGX_DEBUG_POSTFIX})
  if(AGX_64BIT_ARCHITECTURE)
    set(AGX_BUILD_ARCHITECTURE x64)
  else()
    set(AGX_BUILD_ARCHITECTURE x86)
  endif()
endif(MSVC)

if(NOT AGX_DIR)
  if(DEFINED ENV{AGX_DIR})
    set(agx_dir $ENV{AGX_DIR})
  elseif(MSVC)
    file(GLOB agx_dirs "c:/Users/$ENV{USERNAME}/AppData/Local/Algoryx/AGX-?.*.?.?" "c:/Program Files/Algoryx/AGX-?.*.?.?")
    if(agx_dirs)
      list(REVERSE agx_dirs) # Prefer a newer version
      list(GET agx_dirs 0 agx_dir)
    endif()
  endif()
  if(agx_dir)
    set(AGX_DIR ${agx_dir} CACHE PATH "set the top directory of the AGX Dynamics library" FORCE)
  endif()
endif()

if(NOT AGX_DIR)
  message(FATAL_ERROR "AGX_DIR must be set to build AGXDynamicsPlugin.")
elseif(NOT EXISTS ${AGX_DIR})
  message(FATAL_ERROR "The directory \"${AGX_DIR}\" specified in AGX_DIR is not found.")
endif()

set(CMAKE_MODULE_PATH "${AGX_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

set(cxx_standard ${CMAKE_CXX_STANDARD})

include(AGXUtils)
include(AGX${CMAKE_SYSTEM_NAME}Specific)
include(AGXCompilerFlags )

# Restore the original C++ version
set(CMAKE_CXX_STANDARD ${cxx_standard})

# Now locate AgX
find_package(AGX REQUIRED QUIET)

if(AGX_FOUND)
  message(STATUS "AGX Dynamics found in \"${AGX_DIR}\".")
endif()

#if(MSVC)
  #if(NOT CMAKE_CXX_STANDARD LESS 17)
  #  message(WARNING "AGX with VC++ does not support C++17 or later. C++14 is used in this project.")
  #  set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version number. AGX with VC++ does not support C++17 or later." FORCE)
  #endif()
#endif()

include_directories(${AGX_INCLUDE_DIR} ${AGX_INCLUDE_DIR}/external)
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

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})

set(agx_libraries optimized ${AGX_LIBRARIES_RELEASE})
if(AGX_LIBRARIES_DEBUG)
  set(agx_libraries ${agx_libraries} debug ${AGX_LIBRARIES_DEBUG})
else()
  set(agx_libraries ${agx_libraries} debug ${AGX_LIBRARIES_RELEASE})
endif()

target_link_libraries(${target} PUBLIC CnoidBodyPlugin ${agx_libraries})

add_subdirectory(AGXBodyExtensionPlugin)

if(ENABLE_PYTHON AND USE_PYBIND11)
  add_subdirectory(pybind11)
endif()

if(WIN32 AND INSTALL_AGX_RUNTIME)
  find_path(AGX_BIN_DIR NAMES agxCore.dll PATHS ${AGX_DIR}/bin PATH_SUFFIXES x64 NO_DEFAULT_PATH)
  install_runtime_dlls(
    ${AGX_BIN_DIR}
    agxCore.dll agxPhysics.dll agxSabre.dll agxVehicle.dll
    OPTIONAL
    zlib.dll libpng.dll websockets.dll colamd.dll
    # AGX 2.31 or later does not require the followings
    agx-assimp-vc142-mt.dll glew.dll
    )
endif()
