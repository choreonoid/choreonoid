
# @author Shin'ichiro Nakaoka

option(BUILD_PHENOMENON_PLUGIN "Building PhenomenonPlugin" OFF)

if(NOT BUILD_PHENOMENON_PLUGIN)
  return()
endif()

set(sources
  PhenomenonPlugin.cpp
  SceneFountain.cpp
  )

set(headers

  )

set(target CnoidPhenomenonPlugin)

make_gettext_mofiles(${target} mofiles)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})

target_link_libraries(${target} CnoidBase CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")
