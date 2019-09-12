option(BUILD_MANIPULATOR_PLUGIN "Building ManipulatorPlugin" OFF)
if(NOT BUILD_MANIPULATOR_PLUGIN)
  return()
endif()

set(target CnoidManipulatorPlugin)

set(sources
  ManipulatorPlugin.cpp
  ManipulatorFrame.cpp
  ManipulatorPosition.cpp
  ManipulatorProgram.cpp
  ManipulatorStatements.cpp
  BodyManipulatorManager.cpp
  ManipulatorProgramItemBase.cpp
  ManipulatorControllerItemBase.cpp
  ManipulatorProgramViewBase.cpp
  ManipulatorStatementViewBase.cpp
  )

set(headers
  ManipulatorFrame.h
  ManipulatorPosition.h
  ManipulatorProgram.h
  ManipulatorStatements.h
  BodyManipulatorManager.h
  ManipulatorProgramItemBase.h
  ManipulatorControllerItemBase
  ManipulatorProgramViewBase.h
  ManipulatorStatementViewBase.h
  exportdecl.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)

apply_common_setting_for_library(${target} "${headers}")