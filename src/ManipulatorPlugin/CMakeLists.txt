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
  BasicMprStatements.cpp
  MprVariable.cpp
  MprVariableSet.cpp
  MprVariableList.cpp
  MprVariableSetGroup.cpp
  MprProgramItemBase.cpp
  MprControllerItemBase.cpp
  MprVariableListItemBase.cpp
  MprProgramViewBase.cpp
  MprProgramViewDelegates.cpp
  MprStatementViewBase.cpp
  MprVariableListViewBase.cpp
  )

set(headers
  MprProgram.h
  MprPositionList.h
  MprStatementRegistration.h
  MprStatement.h
  BasicMprStatements.h
  MprPositionStatement.h
  MprVariable.h
  MprVariableSet.h
  MprVariableList.h
  MprVariableSetGroup.h
  MprProgramItemBase.h
  MprControllerItemBase.h
  MprVariableListItemBase.h
  MprProgramViewBase.h
  MprStatementViewBase.h
  MprVariableListViewBase.h
  exportdecl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBodyPlugin)
