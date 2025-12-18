set(include_dirs ${CHOREONOID_SDK_INCLUDE_DIRS})
list(REMOVE_DUPLICATES include_dirs)

set(library_dirs ${CHOREONOID_SDK_LIBRARY_DIRS})
list(REMOVE_DUPLICATES library_dirs)

get_directory_property(compile_definitions COMPILE_DEFINITIONS)

set(modules
  ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfig.cmake
  ChoreonoidBasicBuildFunctions.cmake
  ChoreonoidFindQt.cmake
  )

set(util_dependencies fmt ${FILESYSTEM_LIBRARY})

if(UNIX)
  set(CHOREONOID_USE_SUBSYSTEM_CONSOLE false)
elseif(MSVC)
  if(ENABLE_GETTEXT)
    list(APPEND util_dependencies ${GETTEXT_LIBRARIES})
  endif()
endif()

if(ENABLE_PYTHON)
  list(APPEND modules ChoreonoidPythonBuildFunctions.cmake)
endif()

if(ENABLE_CORBA AND BUILD_CORBA_PLUGIN)
  set(is_corba_enabled true)
  list(APPEND modules ChoreonoidCorbaBuildFunctions.cmake)
endif()

install(FILES ${modules} DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR})
if(UNIX)
  # Create symlink for backward compatibility with lowercase package name.
  # The official usage is find_package(Choreonoid) with capital 'C',
  # but this symlink allows find_package(choreonoid) to work as well.
  
  # Create symlink at build time and install it
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ChoreonoidConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/choreonoid-config.cmake
  )
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/choreonoid-config.cmake
          DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR})
endif()

configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)

# Generate setup scripts for environment configuration
# Only generate for non-standard installation paths where environment setup is needed
if(UNIX)
  # Don't generate setup script for standard system paths (/usr, /usr/local)
  # These locations already have PATH and library paths configured
  if(NOT CMAKE_INSTALL_PREFIX MATCHES "^/usr(/local)?$")
    configure_file(setup.bash.in ${CMAKE_CURRENT_BINARY_DIR}/setup.bash @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.bash
            DESTINATION .
            PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                        GROUP_READ GROUP_EXECUTE
                        WORLD_READ WORLD_EXECUTE)
  endif()
endif()
