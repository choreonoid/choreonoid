
# @author Shin'ichiro Nakaoka

option(BUILD_SIMPLE_CONTROLLER_PLUGIN "Building SimpleControllerPlugin" ON)

if(NOT BUILD_SIMPLE_CONTROLLER_PLUGIN)
  return()
endif()

# set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(library)

set(plugin CnoidSimpleControllerPlugin)
set(sources
  SimpleControllerPlugin.cpp
  SimpleControllerItem.cpp
)
set(headers
  SimpleControllerItem.h
  exportdecl.h
  gettext.h
)
#make_gettext_mofiles(${plugin} mofiles)
add_cnoid_plugin(${plugin} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${plugin} CnoidBodyPlugin CnoidSimpleController)
apply_common_setting_for_plugin(${plugin} "${headers}")

if(QT5)
  qt5_use_modules(${plugin} Gui Widgets)
endif()

if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
