if(WIN32)
  option(BUILD_PYTHON_PLUGIN "Building Python Plugin" OFF)
else()
  option(BUILD_PYTHON_PLUGIN "Building Python Plugin" ON)
endif()

if(NOT BUILD_PYTHON_PLUGIN)
  return()
elseif(NOT ENABLE_PYTHON)
  message(FATAL_ERROR "PythonPlugin needs to ENABLE_PYTHON")
endif()

set(target CnoidPythonPlugin)

set(sources
  PythonPlugin.cpp
  PythonExecutor.cpp
  PythonScriptItem.cpp
  PythonScriptItemImpl.cpp
  PythonConsoleView.cpp
  )

set(headers
  PythonExecutor.h
  PythonScriptItem.h
  PythonScriptItemImpl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} CnoidBase CnoidPyUtil)

configure_file(rbimporter.py ${PROJECT_BINARY_DIR}/${CNOID_PYTHON_SUBDIR}/cnoid/ COPYONLY)
install(FILES rbimporter.py DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid)
