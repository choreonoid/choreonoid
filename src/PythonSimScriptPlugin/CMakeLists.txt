
# @author Shin'ichiro Nakaoka

if(WIN32)
  option(BUILD_PYTHON_SIM_SCRIPT_PLUGIN "Building the PythonSimScript plugin" OFF)
else()
  option(BUILD_PYTHON_SIM_SCRIPT_PLUGIN "Building the PythonSimScript plugin" ON)
endif()

if(NOT BUILD_PYTHON_SIM_SCRIPT_PLUGIN)
  return()
else()
  if(USE_PYTHON3 AND NOT BUILD_PYTHON3_PLUGIN)
    message(FATAL_ERROR "The PythonSimScript plugin requires the Python3 plugin")
  elseif(NOT USE_PYTHON3 AND NOT BUILD_PYTHON_PLUGIN)
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

if(ENABLE_PYTHON)
  if(USE_PYTHON3)
    add_subdirectory(python3)
  else()
    add_subdirectory(python)
  endif()
endif()
