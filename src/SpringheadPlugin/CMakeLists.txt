
# @author Shin'ichiro Nakaoka

option(BUILD_SPRINGHEAD_PLUGIN "Building SpringheadPlugin" OFF)

if(NOT BUILD_SPRINGHEAD_PLUGIN)
  return()
endif()

# set(CMAKE_BUILD_TYPE Debug)

set(SPRINGHEAD_DIR ${SPRINGHEAD_DIR} CACHE PATH "set the top directory of Springhead")

if(UNIX)
  if(NOT SPRINGHEAD_DIR)
    pkg_check_modules(SPRINGHEAD REQUIRED springhead)
    set(SPRINGHEAD_LIBRARIES libSpringhead.a)
  else()
    set(SPRINGHEAD_INCLUDE_DIRS ${SPRINGHEAD_DIR}/include ${SPRINGHEAD_DIR}/src)
    set(SPRINGHEAD_LIBRARY_DIRS ${SPRINGHEAD_DIR}/lib/linux)
    set(SPRINGHEAD_LIBRARIES libSpringhead.a)
  endif()
elseif(MSVC)
  if(NOT SPRINGHEAD_DIR)
    message(FATAL_ERROR "Please specify the directory of Springhead to SPRINGHEAD_DIR.")
  else()
    set(SPRINGHEAD_INCLUDE_DIRS ${SPRINGHEAD_DIR}/include ${SPRINGHEAD_DIR}/src)
    set(SPRINGHEAD_LIBRARY_DIRS ${SPRINGHEAD_DIR}/lib/win64)
    set(SPRINGHEAD_LIBRARIES optimized Springhead14.0x64.lib debug Springhead14.0Dx64.lib)
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


  
  
