option(BUILD_SPRINGHEAD_PLUGIN "Building SpringheadPlugin" OFF)
mark_as_advanced(BUILD_SPRINGHEAD_PLUGIN)

if(NOT BUILD_SPRINGHEAD_PLUGIN)
  return()
endif()

set(SPRINGHEAD_DIR             ${SPRINGHEAD_DIR}             CACHE PATH     "set the top directory of Springhead")
set(SPRINGHEAD_LIBRARY_DEBUG   ${SPRINGHEAD_LIBRARY_DEBUG}   CACHE FILEPATH "set the debug library path of Springhead")
set(SPRINGHEAD_LIBRARY_RELEASE ${SPRINGHEAD_LIBRARY_RELEASE} CACHE FILEPATH "set the release library path of Springhead")

if(UNIX)
  if(NOT SPRINGHEAD_DIR)
    pkg_check_modules(SPRINGHEAD REQUIRED springhead)
    set(SPRINGHEAD_LIBRARIES libSpringhead.a)
  else()
    set(SPRINGHEAD_INCLUDE_DIRS ${SPRINGHEAD_DIR}/core/include ${SPRINGHEAD_DIR}/core/src)
    set(SPRINGHEAD_LIBRARY_DIRS ${SPRINGHEAD_DIR}/generated/lib/linux)
    set(SPRINGHEAD_LIBRARIES ${SPRINGHEAD_LIBRARY_RELEASE})
  endif()
elseif(MSVC)
  if(NOT SPRINGHEAD_DIR)
    message(FATAL_ERROR "Please specify the directory of Springhead to SPRINGHEAD_DIR.")
  else()
    set(SPRINGHEAD_INCLUDE_DIRS ${SPRINGHEAD_DIR}/core/include ${SPRINGHEAD_DIR}/core/src)
    set(SPRINGHEAD_LIBRARY_DIRS ${SPRINGHEAD_DIR}/generated/lib/win64)
    set(SPRINGHEAD_LIBRARIES optimized ${SPRINGHEAD_LIBRARY_RELEASE} debug ${SPRINGHEAD_LIBRARY_DEBUG})
  endif()
endif()

include_directories(${SPRINGHEAD_INCLUDE_DIRS})
link_directories(${SPRINGHEAD_LIBRARY_DIRS})

set(target CnoidSpringheadPlugin)

set(sources
  SpringheadPlugin.cpp
  SpringheadSimulatorItem.cpp
  #SpringheadCollisionDetector.cpp # option
)

set(headers
  )

make_gettext_mofiles(${target} mofiles) 
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin ${SPRINGHEAD_LIBRARIES})
