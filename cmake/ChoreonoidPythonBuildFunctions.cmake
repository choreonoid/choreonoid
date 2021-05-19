function(choreonoid_add_python_module)
  
  set(target ${ARGV0})
  string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})
  set(sources ${ARGV})
  list(REMOVE_AT sources 0)

  add_library(${target} SHARED ${sources})
  if(TARGET Choreonoid::CnoidPyUtil)
    target_link_libraries(${target} Choreonoid::CnoidPyUtil)
  else()
    target_link_libraries(${target} CnoidPyUtil)
  endif()
    
  if(MSVC)
    set_target_properties(${target} PROPERTIES SUFFIX .pyd)
    target_link_options(${target} PRIVATE "/NODEFAULTLIB:LIBCMT")
  else()
    if(CHOREONOID_DEFAULT_FVISIBILITY_HIDDEN)
      target_compile_options(${target} PRIVATE "-fvisibility=hidden")
    endif()
    if(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
      if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	target_compile_options(${target} PRIVATE "-flto;-fno-fat-lto-objects")
	set(link_lto_flags "-flto")
      elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	target_compile_options(${target} PRIVATE "-flto=thin")
	set(link_lto_flags "-flto=thin")
      endif()
      # TODO: Use target_link_options when CMake 3.13 or later is available for Ubuntu
      get_target_property(existing_link_flags ${target} LINK_FLAGS)
      if(NOT existing_link_flags)
	set(existing_link_flags "")
      endif()
      set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} ${link_lto_flags}")

      #add_custom_command(TARGET ${target} POST_BUILD COMMAND strip $<TARGET_FILE:${target}>)
    endif()
  endif()

  set_target_properties(${target}  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid
    RUNTIME_OUTPUT_NAME ${module}
    LIBRARY_OUTPUT_NAME ${module}
    PREFIX "")

  if(CHOREONOID_ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/../..:$ORIGIN/../../..")
  endif()

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid
    LIBRARY DESTINATION ${CHOREONOID_PYTHON_SUBDIR}/cnoid)
endfunction()

# Deprecated.
function(add_cnoid_python_module)
  choreonoid_add_python_module(${ARGV})
endfunction()
