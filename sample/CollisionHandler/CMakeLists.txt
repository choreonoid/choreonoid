
#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_COLLISION_HANDLER_SAMPLE "Building a collision handler sample" OFF)
mark_as_advanced(BUILD_COLLISION_HANDLER_SAMPLE)

if(NOT BUILD_COLLISION_HANDLER_SAMPLE)
  return()
endif()

set(sources
  SpringDamperContactPlugin.cpp
)

set(target CnoidSpringDamperContactPlugin)
choreonoid_add_plugin(${target} SHARED ${sources})
target_link_libraries(${target} CnoidBodyPlugin)
