
set(include_dirs
  ${CNOID_DIR}/${CNOID_HEADER_SUBDIR}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  )

list(REMOVE_DUPLICATES include_dirs)

set(library_dirs ${CNOID_DIR}/lib ${CNOID_DIR}/${CNOID_PLUGIN_SUBDIR})
list(REMOVE_DUPLICATES library_dirs)

set(definitions -DQT_NO_KEYWORDS -DQT_NO_OPENGL_ES_2)

configure_file(ChoreonoidConfig.cmake.in ChoreonoidConfig.cmake @ONLY)
if(INSTALL_SDK)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ChoreonoidConfig.cmake DESTINATION lib/cmake/Choreonoid)
endif()
