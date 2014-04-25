
foreach(file choreonoid.pc choreonoid-body-plugin.pc)
  configure_file(${file}.in ${PROJECT_BINARY_DIR}/lib/pkgconfig/${file} @ONLY)
  if(INSTALL_SDK)
    install(FILES ${PROJECT_BINARY_DIR}/lib/pkgconfig/${file} DESTINATION lib/pkgconfig)
  endif()
endforeach()

