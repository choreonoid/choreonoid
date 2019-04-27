
if(NOT UNIX)
  return()
endif()

option(BUILD_SDF_PLUGIN "Building SDFPlugin" OFF)


if(NOT BUILD_SDF_PLUGIN)
  return()
endif()

set(SDFORMAT_DIR ${SDFORMAT_DIR} CACHE PATH "set the top directory of the SDFORMAT")
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

set(OGRE_DIR ${OGRE_DIR} CACHE PATH "set the top directory of the OGRE ")
if(UNIX)
  if(NOT OGRE_DIR)
    pkg_check_modules(OGRE REQUIRED OGRE)
  endif()
endif()

if(NOT OGRE_FOUND)
  set(OGRE_LIBRARIES OgreMain pthread)
  set(OGRE_INCLUDE_DIRS ${OGRE_DIR}/include)
  set(OGRE_LIBRARY_DIRS ${OGRE_DIR}/lib)
endif()

set(target CnoidSDFPlugin)

include_directories(
  ${SDFORMAT_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

link_directories(
  ${SDFORMAT_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
)

set(sources
  SDFPlugin.cpp
  SDFBodyLoader.cpp
  SDFLoaderPseudoGazeboColor.cpp
)

set(headers
  SDFBodyLoader.h
  SDFLoaderPseudoGazeboColor.h
)

make_gettext_mofiles(${target} mofiles) 
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin CnoidAssimpPlugin ${SDFORMAT_LIBRARIES} ${OGRE_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
