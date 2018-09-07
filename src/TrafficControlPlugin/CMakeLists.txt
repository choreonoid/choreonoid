option(BUILD_TRAFFIC_CONTROL_PLUGIN "Building a Traffic Contorol plugin" OFF)

if(NOT BUILD_TRAFFIC_CONTROL_PLUGIN)
  return()
endif()

set(sources
  TrafficControlPlugin.cpp
  TrafficControlShare.cpp
  TCSimulatorItem.cpp 
  DynamicTCSimulatorItem.cpp 
  )

set(target CnoidTrafficControlPlugin)
make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target})
