# @author Shin'ichiro Nakaoka

if(WIN32)
  option(BUILD_PYTHON_SIM_SCRIPT_PLUGIN "Building the PythonSimScript plugin" OFF)
else()
  option(BUILD_PYTHON_SIM_SCRIPT_PLUGIN "Building the PythonSimScript plugin" ON)
endif()

if(NOT BUILD_PYTHON_SIM_SCRIPT_PLUGIN)
  return()
else()
  if(NOT BUILD_PYTHON_PLUGIN)
    message(FATAL_ERROR "The PythonSimScript plugin requires the Python plugin")
  endif()
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
