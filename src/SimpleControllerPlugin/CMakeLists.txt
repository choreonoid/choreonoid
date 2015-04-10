
# @author Shin'ichiro Nakaoka

option(BUILD_SIMPLE_CONTROLLER_PLUGIN "Building SimpleControllerPlugin" ON)

if(NOT BUILD_SIMPLE_CONTROLLER_PLUGIN)
  return()
endif()

# set(CMAKE_BUILD_TYPE Debug)

set(target CnoidSimpleControllerPlugin)
set(sources
  SimpleControllerPlugin.cpp
  SimpleControllerItem.cpp
)
set(headers
  SimpleControllerItem.h
  SimpleController.h
  exportdecl.h
  gettext.h
)
make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")

if(QT5)
  qt5_use_modules(${target} Gui Widgets)
endif()

function(add_cnoid_simple_controller)

  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})

  target_link_libraries(${target} CnoidBody)

  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
    PREFIX "")
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()


file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller)
install(DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller
  FILES_MATCHING PATTERN "*" EXCLUDE)

if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
