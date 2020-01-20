set(library_dirs ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_LIB_SUBDIR} ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_PLUGIN_SUBDIR})
list(REMOVE_DUPLICATES library_dirs)

#get_directory_property(definitions COMPILE_DEFINITIONS)
#message(STATUS "definitions: ${definitions}")

set(definitions -DEIGEN_NO_DEBUG -DQT_NO_DEBUG -DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)

set(modules
  ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfig.cmake
  ChoreonoidBasicBuildFunctions.cmake)

if(ENABLE_PYTHON)
  list(APPEND modules ChoreonoidPythonBuildFunctions.cmake)
endif()

if(ENABLE_CORBA AND BUILD_CORBA_PLUGIN)
  set(is_corba_enabled true)
  list(APPEND modules ChoreonoidCorbaBuildFunctions.cmake)
endif()

if(INSTALL_SDK)
  install(FILES ${modules} DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR})
  if(UNIX)
    string(CONCAT code
      "execute_process(COMMAND ${CMAKE_COMMAND} "
      "-E create_symlink ChoreonoidConfig.cmake choreonoid-config.cmake "
      "WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_CMAKE_CONFIG_SUBDIR})")
    install(CODE ${code})
  endif()
endif()

configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)
