option(BUILD_MANIPULATOR_PLUGIN "Building ManipulatorPlugin" OFF)
if(NOT BUILD_MANIPULATOR_PLUGIN)
  return()
endif()

set(target CnoidManipulatorPlugin)

set(sources
  ManipulatorPlugin.cpp
  MprPosition.cpp
  MprPositionList.cpp
  MprProgram.cpp
  MprStatementRegistration.cpp
  MprStatement.cpp
  MprBasicStatements.cpp
  MprVariable.cpp
  MprVariableList.cpp
  MprProgramItemBase.cpp
  MprControllerItemBase.cpp
  MprVariableListItemBase.cpp
  MprProgramViewBase.cpp
  MprProgramViewDelegates.cpp
  MprStatementViewBase.cpp
  MprPositionListView.cpp
  MprVariableListViewBase.cpp
  )

set(headers
  MprProgram.h
  MprPositionList.h
  MprStatementRegistration.h
  MprStatement.h
  MprBasicStatements.h
  MprPositionStatement.h
  MprVariable.h
  MprVariableList.h
  MprProgramItemBase.h
  MprControllerItemBase.h
  MprVariableListItemBase.h
  MprProgramViewBase.h
  MprStatementViewBase.h
  MprPositionListView.h
  MprVariableListViewBase.h
  exportdecl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin)
