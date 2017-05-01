
if(NOT UNIX)
  return()
endif()

option(BUILD_ASSIMP_PLUGIN "Building ASSIMPPlugin" ON)

if(NOT BUILD_ASSIMP_PLUGIN)
  return()
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

if(NOT ASSIMP_FOUND )
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_DIR}/include)
  set(ASSIMP_LIBRARY_DIRS ${ASSIMP_DIR}/lib)
endif()

set(target CnoidAssimpPlugin)

include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})

set(sources
  AssimpPlugin.cpp
  AssimpSceneLoader.cpp
)

set(headers )

make_gettext_mofiles(${target} mofiles) 
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidUtil ${ASSIMP_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
