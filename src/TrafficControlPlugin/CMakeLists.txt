if(MSVC)
  return()
endif()

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
choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)
