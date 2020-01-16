
# @author Shin'ichiro Nakaoka

option(BUILD_SPRINGHEAD_PLUGIN "Building SpringheadPlugin" OFF)

if(NOT BUILD_SPRINGHEAD_PLUGIN)
  return()
endif()

# set(CMAKE_BUILD_TYPE Debug)

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
  SpringheadCollisionDetector.cpp
)

set(headers
  )

make_gettext_mofiles(${target} mofiles) 
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin ${SPRINGHEAD_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")


  
  
