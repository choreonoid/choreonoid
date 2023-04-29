#set(CMAKE_BUILD_TYPE Debug)

if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_AGX_BODYEXTENSION_PLUGIN "Building a AGXBodyExtensionPlugin" OFF)
if(NOT BUILD_AGX_BODYEXTENSION_PLUGIN)
  return()
endif()

set(target CnoidAGXBodyExtensionPlugin)

set(sources
  AGXBodyExtensionPlugin.cpp
  AGXBodyExtensionSample.cpp
  AGXBreakableJoint.cpp
  AGXMagneticJoint.cpp
  AGXWire.cpp
)

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources})
target_link_libraries(${target} PUBLIC CnoidAGXDynamicsPlugin)
