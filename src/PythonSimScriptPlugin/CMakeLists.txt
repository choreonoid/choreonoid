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

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin CnoidPythonPlugin)
apply_common_setting_for_plugin(${target} "${headers}")

add_subdirectory(pybind11)
