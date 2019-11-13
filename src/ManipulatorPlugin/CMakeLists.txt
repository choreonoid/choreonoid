option(BUILD_MANIPULATOR_PLUGIN "Building ManipulatorPlugin" OFF)
if(NOT BUILD_MANIPULATOR_PLUGIN)
  return()
endif()

set(target CnoidManipulatorPlugin)

set(sources
  ManipulatorPlugin.cpp
  ManipulatorPosition.cpp
  ManipulatorPositionList.cpp
  ManipulatorProgram.cpp
  ManipulatorStatement.cpp
  BasicManipulatorStatements.cpp
  ManipulatorVariable.cpp
  ManipulatorVariableSet.cpp
  ManipulatorVariableList.cpp
  ManipulatorVariableSetGroup.cpp
  ManipulatorProgramItemBase.cpp
  ManipulatorControllerItemBase.cpp
  ManipulatorVariableListItemBase.cpp
  ManipulatorProgramViewBase.cpp
  ManipulatorProgramViewDelegates.cpp
  ManipulatorStatementViewBase.cpp
  ManipulatorVariableListViewBase.cpp
  )

set(headers
  ManipulatorProgram.h
  ManipulatorPositionList.h
  ManipulatorStatement.h
  BasicManipulatorStatements.h
  ManipulatorVariable.h
  ManipulatorVariableSet.h
  ManipulatorVariableList.h
  ManipulatorVariableSetGroup.h
  ManipulatorProgramItemBase.h
  ManipulatorControllerItemBase
  ManipulatorVariableListItemBase.cpp
  ManipulatorProgramViewBase.h
  ManipulatorStatementViewBase.h
  ManipulatorVariableListViewBase.h
  exportdecl.h
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)

apply_common_setting_for_plugin(${target} "${headers}")
