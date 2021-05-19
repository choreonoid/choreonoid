if(NOT ENABLE_PYTHON OR NOT ENABLE_GUI)
  return()
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
target_link_libraries(${target} CnoidPyUtil)

configure_file(rbimporter.py ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid/ COPYONLY)
install(FILES rbimporter.py DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid)

add_subdirectory(pybind11)
