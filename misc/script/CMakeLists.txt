
configure_file(make_header_public.rb.in make_header_public.rb @ONLY)
configure_file(make_binary_package.sh.template make_binary_package.sh @ONLY)
configure_file(make_src_archive.sh.in make_src_archive.sh @ONLY)

if(WIN32)
  configure_file(install_win32_deps.rb.template install_win32_deps.rb)
endif()
