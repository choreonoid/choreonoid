set(pkgconfig_dir ${PROJECT_BINARY_DIR}/lib/pkgconfig)

if(EIGEN_FOUND)
  set(PKGCONFIG_EIGEN_REQUIREMENT "eigen3 >= 3.2.7")
else()
  unset(PKGCONFIG_EIGEN_REQUIREMENT)
endif()

if(USE_QT5)
  set(PKGCONFIG_QT_REQUIREMENTS "Qt5Core >= 5.2.0 Qt5Gui >= 5.2.0 Qt5OpenGL >= 5.2.0 Qt5Network >= 5.2.0")
else()
  set(PKGCONFIG_QT_REQUIREMENTS "QtCore >= 4.7.0 QtGui >= 4.7.0 QtOpenGL >= 4.7.0 QtNetwork >= 4.7.0")
endif()

foreach(file choreonoid-util.pc choreonoid-base.pc choreonoid-body.pc choreonoid-body-plugin.pc)
  configure_file(${file}.in ${pkgconfig_dir}/${file} @ONLY)
  if(INSTALL_SDK)
    install(FILES ${pkgconfig_dir}/${file} DESTINATION lib/pkgconfig)
  endif()
endforeach()

# choreonoid.pc is replaced with choreonoid-base.pc, 
# but it is still generated for the backward compatibility
configure_file(choreonoid-base.pc.in ${pkgconfig_dir}/choreonoid.pc @ONLY)
if(INSTALL_SDK)
  install(FILES ${pkgconfig_dir}/choreonoid.pc DESTINATION lib/pkgconfig)
endif()
