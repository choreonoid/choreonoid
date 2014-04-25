
configure_file(make_header_public.rb.in make_header_public.rb @ONLY)
configure_file(make_binary_package.sh.template ${CMAKE_CURRENT_SOURCE_DIR}/make_binary_package.sh @ONLY)
configure_file(make_src_archive.sh.in ${CMAKE_CURRENT_SOURCE_DIR}/make_src_archive.sh @ONLY)

if(WIN32)
  configure_file(install_win32_deps.rb.template ${CMAKE_CURRENT_SOURCE_DIR}/install_win32_deps.rb)
endif()
