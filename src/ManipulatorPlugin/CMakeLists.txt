option(BUILD_MANIPULATOR_PLUGIN "Building ManipulatorPlugin" OFF)
if(NOT BUILD_MANIPULATOR_PLUGIN)
  return()
endif()

set(target CnoidManipulatorPlugin)

set(sources
  ManipulatorPlugin.cpp
  ManipulatorFrame.cpp
  ManipulatorPosition.cpp
  BodyManipulatorManager.cpp
  )

set(headers
  ManipulatorFrame.h
  ManipulatorPosition.h
  BodyManipulatorManager.h
  exportdecl.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)

apply_common_setting_for_library(${target} "${headers}")
