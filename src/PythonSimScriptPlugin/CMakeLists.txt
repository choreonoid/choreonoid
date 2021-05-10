if(NOT ENABLE_PYTHON OR NOT ENABLE_GUI)
  return()
endif()

set(target CnoidPythonSimScriptPlugin)

set(sources
  PythonSimScriptPlugin.cpp
  PythonSimScriptItem.cpp
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin CnoidPythonPlugin)

add_subdirectory(pybind11)
