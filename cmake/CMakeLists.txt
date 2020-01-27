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

if(ENABLE_PYTHON)
  list(APPEND modules ChoreonoidPythonBuildFunctions.cmake)
endif()

if(ENABLE_CORBA AND BUILD_CORBA_PLUGIN)
  set(is_corba_enabled true)
  list(APPEND modules ChoreonoidCorbaBuildFunctions.cmake)
endif()

install(FILES ${modules} DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR})
if(UNIX)
  string(CONCAT code
    "execute_process(COMMAND ${CMAKE_COMMAND} "
    "-E create_symlink ChoreonoidConfig.cmake choreonoid-config.cmake "
    "WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_CMAKE_CONFIG_SUBDIR})")
  install(CODE ${code})
endif()

configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)
