function(CHOREONOID_LUA_ADD_MODULE target)
  string(REGEX REPLACE "^Lua(.+)$" "\\1" module ${target})
  set(sources ${ARGN})

  add_library(${target} SHARED ${sources})

  set_target_properties(${target} PROPERTIES VERSION ${CHOREONOID_VERSION})
  CHOREONOID_SET_TARGET_COMMON_PROPERTIES(${target})
  CHOREONOID_SET_HEADER_FILES(${sources} INSTALL_HEADERS)
    
  set_target_properties(${target}  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_LUA_SUBDIR}/cnoid
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_LUA_SUBDIR}/cnoid
    RUNTIME_OUTPUT_NAME ${module}
    LIBRARY_OUTPUT_NAME ${module}
    PREFIX "")

  if(CHOREONOID_INSTALL_SDK AND MSVC)
    install(TARGETS ${target} ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_LUA_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CHOREONOID_LUA_SUBDIR}/cnoid CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()
endfunction()

# Deprecated
function(add_cnoid_lua_module)
  CHOREONOID_LUA_ADD_MODULE(${ARGV})
endfunction()

# Deprecated
function(apply_common_setting_for_lua_module target)
endfunction()
