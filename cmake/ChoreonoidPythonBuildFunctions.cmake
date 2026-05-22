function(choreonoid_add_python_module)

  set(target ${ARGV0})
  string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})
  set(sources ${ARGV})
  list(REMOVE_AT sources 0)

  if(CHOREONOID_PYTHON_BINDING_BACKEND STREQUAL "nanobind")

    # nanobind_add_module creates an "add_library(... MODULE)" target and takes
    # care of visibility, size optimization, stripping, the Python extension
    # suffix and linking the nanobind runtime. It links its own dependencies
    # using the keyword signature, so every target_link_libraries call for this
    # target must use the keyword (PUBLIC/PRIVATE) signature as well.
    #
    # FREE_THREADED is passed for a free-threaded (no-GIL) interpreter so that
    # the module declares Py_MOD_GIL_NOT_USED and the interpreter keeps the GIL
    # disabled. Whether to pass it is decided from CHOREONOID_PYTHON_FREE_THREADED
    # rather than from nanobind's own NB_FREE_THREADED: the former is queried from
    # the interpreter (so it is available regardless of the binding backend) and
    # is exported through ChoreonoidConfig.cmake, so external module builds use
    # the same setting that the Choreonoid core was configured with.
    #
    # NB_SHARED (rather than NB_STATIC) makes the nanobind core runtime live in a
    # single shared library that is shared by every Python module and by the
    # CnoidPy* helper libraries (e.g. CnoidPyBase, which itself compiles code that
    # uses the runtime). With NB_STATIC each module would embed its own copy of
    # the runtime, and such a helper shared library would have no shared runtime
    # library from which to resolve those symbols.
    if(CHOREONOID_PYTHON_FREE_THREADED)
      nanobind_add_module(${target} NB_SHARED FREE_THREADED ${sources})
    else()
      nanobind_add_module(${target} NB_SHARED ${sources})
    endif()
    # No CnoidPyUtil link in nanobind mode: the headers (<cnoid/PyUtil> etc.)
    # are reachable through the global include path, and the nanobind runtime is
    # already linked by nanobind_add_module(). See src/Util/python/CMakeLists.txt.
    # Note: extension modules must not be linked with --no-undefined because the
    # CPython symbols they use are resolved by the interpreter at load time.

  else()

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

    # Check for unresolved symbols in Python modules
    if(UNIX AND NOT APPLE)
      target_link_options(${target} PRIVATE "LINKER:--no-undefined")
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

# Add the Python binding subdirectory of the current module for the active
# binding backend. The pybind11 backend builds the "pybind11" subdirectory and
# the nanobind backend builds the "python" subdirectory. The subdirectory is
# added only when it actually contains a CMakeLists.txt, so modules that have
# not been ported to nanobind yet are silently skipped instead of breaking the
# configuration.
function(choreonoid_add_python_binding_subdirectory)
  if(CHOREONOID_PYTHON_BINDING_BACKEND STREQUAL "nanobind")
    set(subdir python)
  else()
    set(subdir pybind11)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)
    add_subdirectory(${subdir})
  endif()
endfunction()

# Deprecated.
function(add_cnoid_python_module)
  choreonoid_add_python_module(${ARGV})
endfunction()
