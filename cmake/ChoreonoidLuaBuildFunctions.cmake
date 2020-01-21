function(choreonoid_add_lua_module target)

  set(sources ${ARGN})
  list(REMOVE_ITEM sources HEADERS)
  string(REGEX REPLACE "^Lua(.+)$" "\\1" module ${target})
  add_library(${target} SHARED ${sources})

  set_target_properties(${target} PROPERTIES VERSION ${CHOREONOID_VERSION_MAJOR}.${CHOREONOID_VERSION_MINOR})
  choreonoid_set_target_common_properties(${target})
  choreonoid_set_header_files(${ARGN} INSTALL_HEADERS)
    
  set_target_properties(${target}  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_LUA_SUBDIR}/cnoid
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_LUA_SUBDIR}/cnoid
    RUNTIME_OUTPUT_NAME ${module}
    LIBRARY_OUTPUT_NAME ${module}
    PREFIX "")

  if(CHOREONOID_INSTALL_SDK AND MSVC)
    install(TARGETS ${target} ARCHIVE DESTINATION lib)
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_LUA_SUBDIR}/cnoid
      LIBRARY DESTINATION ${CHOREONOID_LUA_SUBDIR}/cnoid)
  endif()
endfunction()

# Deprecated
function(add_cnoid_lua_module)
  choreonoid_add_lua_module(${ARGV})
endfunction()

# Deprecated
function(apply_common_setting_for_lua_module target)
endfunction()
