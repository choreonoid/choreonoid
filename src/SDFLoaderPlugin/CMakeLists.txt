
option(BUILD_SDFLoader_PLUGIN "Building SDFLoaderPlugin" OFF)

if(NOT BUILD_SDFLoader_PLUGIN)
  return()
endif()

if(NOT UNIX)
  return()
endif()

set(SDFORMAT_DIR ${SDFORMAT_DIR} CACHE PATH "set the top directory of the SDFORMAT ")
if(UNIX)
  if(NOT SDFORMAT_DIR)
    pkg_check_modules(SDFORMAT REQUIRED sdformat)
  endif()
elseif(MSVC)
  if(NOT SDFORMAT_DIR)
    message(FATAL_ERROR "Please specify the directory of the SDFORMAT to SDFORMAT_DIR.")
  endif()
endif()

if(NOT SDFORMAT_FOUND)
  set(SDFORMAT_LIBRARIES sdformat)
  set(SDFORMAT_INCLUDE_DIRS ${SDFORMAT_DIR}/include)
  set(SDFORMAT_LIBRARY_DIRS ${SDFORMAT_DIR}/lib)
endif()

set(ASSIMP_DIR ${ASSIMP_DIR} CACHE PATH "set the top directory of the ASSIMP ")
if(UNIX)
  if(NOT ASSIMP_DIR)
    pkg_check_modules(ASSIMP REQUIRED assimp)
  endif()
elseif(MSVC)
  if(NOT ASSIMP_DIR)
    message(FATAL_ERROR "Please specify the directory of the ASSIMP to ASSIMP_DIR.")
  endif()
endif()

if( ASSIMP_FOUND )
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150" )
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
  endif()
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.885" )
    add_definitions(-DASSIMP_EXPORT_API)
  endif()
else()
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_DIR}/include)
  set(ASSIMP_LIBRARY_DIRS ${ASSIMP_DIR}/lib)
endif()


set(target CnoidSDFLoaderPlugin)

include_directories(
  ${SDFORMAT_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

link_directories(
  ${SDFORMAT_LIBRARY_DIRS}
  ${ASSIMP_LIBRARY_DIRS}
)

set(sources
  SDFLoaderPlugin.cpp
  SDFBodyLoader.cpp
  SDFLoaderPseudoGazeboColor.cpp
  SDFSensorConverter.cpp
)

set(headers
  SDFBodyLoader.h
  SDFLoaderPseudoGazeboColor.h
  SDFSensorConverter.h
)

make_gettext_mofiles(${target} mofiles) 
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${SDFORMAT_LIBRARIES} ${ASSIMP_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")


