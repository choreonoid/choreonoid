#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_AGX_SUBSIMULATOR_PLUGIN "Building a AGXSubSimulatorPlugin" OFF)
if(NOT BUILD_AGX_SUBSIMULATOR_PLUGIN)
  return()
endif()

set(sources
  AGXSubSimlatorPlugin.cpp
  AGXSubSimulatorItem.cpp
)

set(headers
  AGXSubSimulatorItem.h
)

set(target CnoidAGXSubSimulatorPlugin)
choreonoid_add_plugin(${target} ${sources} HEADERS ${headers})
#target_link_libraries(${target} CnoidBodyPlugin)
target_link_libraries(${target} CnoidAGXDynamicsPlugin ${AGX_LIBRARIES} ${AGX_LIBRARIES_RELEASE})
