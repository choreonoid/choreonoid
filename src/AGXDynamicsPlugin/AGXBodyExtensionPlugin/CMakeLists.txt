#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_AGX_BODYEXTENSION_PLUGIN "Building a AGXBodyExtensionPlugin" OFF)
if(NOT BUILD_AGX_BODYEXTENSION_PLUGIN)
  return()
endif()

set(sources
  AGXBodyExtensionPlugin.cpp
  AGXBodyExtensionSample.cpp
  AGXBreakableJoint.cpp
  AGXWire.cpp
)

set(headers
)

set(target CnoidAGXBodyExtensionPlugin)
add_cnoid_plugin(${target} SHARED ${sources} ${headers})
target_link_libraries(${target} CnoidAGXDynamicsPlugin ${AGX_LIBRARIES} ${AGX_LIBRARIES_RELEASE})
apply_common_setting_for_plugin(${target})


