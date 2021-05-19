function(choreonoid_add_simple_controller target)

  add_library(${target} SHARED ${ARGN})

  if(TARGET Choreonoid::CnoidBody)
    target_link_libraries(${target} Choreonoid::CnoidBody)
  else()
    target_link_libraries(${target} CnoidBody)
  endif()

  if(MSVC)
    target_link_options(${target} PRIVATE "/NODEFAULTLIB:LIBCMT")
  endif()

  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller
    PREFIX "")

  if(CHOREONOID_ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..:$ORIGIN/../..")
  endif()

  CHOREONOID_SET_HEADER_FILES(${ARGN})

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller
    LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)

endfunction()

# Deprecated
function(add_cnoid_simple_controller)
  choreonoid_add_simple_controller(${ARGV})
endfunction()

# Body handler
function(choreonoid_add_body_handler)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  set_target_properties(${target} PROPERTIES
    PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/bodyhandler)
  if(CHOREONOID_ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..:$ORIGIN/../..")
  endif()
  if(TARGET Choreonoid::CnoidBody)
    target_link_libraries(${target} Choreonoid::CnoidBody)
  else()
    target_link_libraries(${target} CnoidBody)
  endif()
  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/bodyhandler
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/bodyhandler)
endfunction()

# Deprecated
function(add_cnoid_body_handler)
  choreonoid_add_body_handler(${ARGV})
endfunction()

# Body customizer (deprecated)
function(choreonoid_add_body_customizer)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  set_target_properties(${target} PROPERTIES
    PREFIX ""
    COMPILE_DEFINITIONS "CNOID_BODY_CUSTOMIZER"
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/customizer)
  if(CHOREONOID_ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()
  if(TARGET Choreonoid::CnoidUtil)
    target_link_libraries(${target} Choreonoid::CnoidUtil)
  else()
    target_link_libraries(${target} CnoidUtil)
  endif()
  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer)
endfunction()

# Deprecated
function(add_cnoid_body_customizer)
  choreonoid_add_body_customizer(${ARGV})
endfunction()
