
set(include_dirs
  ${CNOID_DIR}/${CNOID_HEADER_SUBDIR}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  )

if(ENABLE_PYTHON AND USE_PYBIND11)
  list(APPEND include_dirs ${PYBIND11_DIR}/include ${PYTHON_INCLUDE_PATH})
endif()

list(REMOVE_DUPLICATES include_dirs)

set(library_dirs ${CNOID_DIR}/lib ${CNOID_DIR}/${CNOID_PLUGIN_SUBDIR})
list(REMOVE_DUPLICATES library_dirs)

set(definitions -DEIGEN_NO_DEBUG -DQT_NO_DEBUG -DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)

configure_file(choreonoid-config.cmake.in choreonoid-config.cmake @ONLY)
if(INSTALL_SDK)
  set(cmake_config_dir share/choreonoid/cmake) # ROS compliant
  #set(cmake_config_dir lib/choreonoid-${CNOID_VERSION}/cmake)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/choreonoid-config.cmake DESTINATION ${cmake_config_dir})
endif()
