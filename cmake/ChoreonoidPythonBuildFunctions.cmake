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
    # LTO is essential for pybind11 to produce reasonably-sized bindings
    # pybind11 documentation states: "pybind11 needs LTO to make good binary code"
    if(NOT ${CMAKE_BUILD_TYPE} MATCHES Debug)
      # Check if LTO is already enabled globally by examining CMAKE_CXX_FLAGS_RELEASE
      # This works both for Choreonoid internal build and external projects
      set(has_global_lto FALSE)
      if(CMAKE_CXX_FLAGS_RELEASE MATCHES "-flto" OR CMAKE_CXX_FLAGS_RELWITHDEBINFO MATCHES "-flto")
        set(has_global_lto TRUE)
      endif()
      
      if(NOT has_global_lto)
        # Enable LTO specifically for Python modules if not globally enabled
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	  target_compile_options(${target} PRIVATE "-flto=auto;-fno-fat-lto-objects")
	  set(link_lto_flags "-flto=auto")
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  target_compile_options(${target} PRIVATE "-flto=thin")
	  set(link_lto_flags "-flto=thin")
        endif()
        if(link_lto_flags)
          # TODO: Use target_link_options when CMake 3.13 or later is available for Ubuntu
          get_target_property(existing_link_flags ${target} LINK_FLAGS)
          if(NOT existing_link_flags)
	    set(existing_link_flags "")
          endif()
          set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} ${link_lto_flags}")
        endif()
      else()
        # When global LTO is enabled, only add -fno-fat-lto-objects for GCC
        # This creates slim objects containing only GIMPLE bytecode, reducing file size
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	  target_compile_options(${target} PRIVATE "-fno-fat-lto-objects")
        endif()
      endif()

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
