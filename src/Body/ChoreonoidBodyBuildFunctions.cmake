function(CHOREONOID_BODY_ADD_SIMPLE_CONTROLLER target)

  add_library(${target} SHARED ${ARGN})

  target_link_libraries(${target} CnoidBody)

  if(MSVC)
    target_link_options(${target} PRIVATE "/NODEFAULTLIB:LIBCMT")
  endif()

  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
    PREFIX "")

  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()

  CHOREONOID_SET_HEADER_FILES(${ARGN})

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()

# Deprecated
function(add_cnoid_simple_controller)
  CHOREONOID_BODY_ADD_SIMPLE_CONTROLLER(${ARGV})
endfunction()
