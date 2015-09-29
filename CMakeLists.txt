# @author 

option(BUILD_SDFLOADER_PLUGIN "Building SDFLoaderPlugin" OFF)

if(NOT BUILD_SDFLOADER_PLUGIN)
  return()
endif()

set(sources
  SDFLoaderPlugin.cpp
  SDFBodyLoader.cpp
  )

set(headers
  SDFBodyLoader.h
  )

set(target CnoidSDFLoaderPlugin)

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

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${SDFORMAT_INCLUDE_DIRS})
link_directories(${SDFORMAT_LIBRARY_DIRS})

add_cnoid_plugin(${target} SHARED ${sources} ${headers})

target_link_libraries(${target} CnoidBase CnoidBody ${SDFORMAT_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
