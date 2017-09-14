# @author Shin'ichiro Nakaoka

option(BUILD_GRXUI_PLUGIN "Building the GrxUI plugin" OFF)

if(NOT BUILD_GRXUI_PLUGIN)
  return()
elseif(NOT BUILD_PYTHON_PLUGIN)
  message(FATAL_ERROR "The GrxUI plugin requires the Python plugin")
endif()

set(target CnoidGrxUIPlugin)

set(sources
  GrxUIPlugin.cpp
  GrxUIMenuView.cpp
  )

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources}  ${mofiles})
target_link_libraries(${target} CnoidPythonPlugin)
apply_common_setting_for_plugin(${target} "${headers}")

add_subdirectory(python)
