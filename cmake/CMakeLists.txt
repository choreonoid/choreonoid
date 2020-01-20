set(library_dirs ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_LIB_SUBDIR} ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_PLUGIN_SUBDIR})
list(REMOVE_DUPLICATES library_dirs)
set(definitions -DEIGEN_NO_DEBUG -DQT_NO_DEBUG -DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)
configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)

set(modules
  ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfig.cmake
  ChoreonoidBasicBuildFunctions.cmake)

if(ENABLE_PYTHON)
  list(APPEND modules ChoreonoidPythonBuildFunctions.cmake)
endif()

if(ENABLE_CORBA)
  list(APPEND modules ChoreonoidCorbaBuildFunctions.cmake)
endif()

if(INSTALL_SDK)
  install(FILES ${modules} DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR})
  if(UNIX)
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_CMAKE_CONFIG_SUBDIR}/ChoreonoidConfig.cmake ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_CMAKE_CONFIG_SUBDIR}/choreonoid-config.cmake)")
  endif()
endif()
