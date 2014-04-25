
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

option(BUILD_SCENARIO_PLUGIN "Building ScenarioPlugin" OFF)

if(NOT BUILD_SCENARIO_PLUGIN)
  return()
elseif(NOT BUILD_MEDIA_PLUGIN)
  message(FATAL_ERROR "ScenarioPlugin requires MediaPlugin")
endif()

set(target CnoidScenarioPlugin)

set(sources
  ScenarioPlugin.cpp
  ScenarioItem.cpp
  ScenarioView.cpp
  )

set(headers
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})

target_link_libraries(${target} CnoidBodyPlugin CnoidMediaPlugin)

apply_common_setting_for_plugin(${target} "${headers}")
