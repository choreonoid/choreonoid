set(library_dirs ${CNOID_DIR}/lib ${CNOID_DIR}/${CNOID_PLUGIN_SUBDIR})
list(REMOVE_DUPLICATES library_dirs)
set(definitions -DEIGEN_NO_DEBUG -DQT_NO_DEBUG -DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)
configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)

if(INSTALL_SDK)
  set(cmake_config_dir share/choreonoid/cmake) # ROS compliant
  #set(cmake_config_dir lib/choreonoid-${CNOID_VERSION}/cmake)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfig.cmake DESTINATION ${cmake_config_dir})
endif()
