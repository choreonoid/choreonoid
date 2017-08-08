
# @author Shin'ichiro Nakaoka

option(BUILD_GRXUIPYTHON3_PLUGIN "Building the GrxUI plugin" OFF)

if(NOT BUILD_GRXUIPYTHON3_PLUGIN)
  return()
elseif(NOT BUILD_PYTHON3_PLUGIN)
  message(FATAL_ERROR "The GrxUI plugin requires the Python3 plugin")
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

if(ENABLE_PYTHON)
  add_subdirectory(python3)
endif()
