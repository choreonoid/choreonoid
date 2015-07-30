
set(target CnoidSimpleController)
set(sources SimpleController.cpp)
set(headers SimpleController.h)
add_cnoid_library(${target} SHARED ${sources} ${headers})
target_link_libraries(${target} CnoidBody)
apply_common_setting_for_library(${target} "${headers}")

function(add_cnoid_simple_controller)

  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})

  target_link_libraries(${target} CnoidSimpleController)

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

install(
  DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
  DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller
  FILES_MATCHING PATTERN "*" EXCLUDE)
