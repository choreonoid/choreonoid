function(choreonoid_compile_corba_idl_to_cpp out_cpp_files out_h_files subdir)

  cmake_parse_arguments(args LOCAL "" "" ${ARGN})
  set(idl_names ${args_UNPARSED_ARGUMENTS})

  set(corba_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
  set(corba_binary_dir ${CMAKE_CURRENT_BINARY_DIR}/${subdir})
  file(MAKE_DIRECTORY ${corba_binary_dir})
  set(idl_flags -C ${corba_binary_dir} -bcxx -Wbh=.hh -Wbs=Sk.cpp -Wba -Wbd=DynSk.cpp -Wbkeep_inc_path
    -I${PROJECT_SOURCE_DIR}/include -I${PROJECT_BINARY_DIR}/include)
  foreach(idl_include_dir ${IDL_INCLUDE_DIRS})
    set(idl_flags ${idl_flags} -I${idl_include_dir})
  endforeach()

  if(args_LOCAL)
    unset(idl_files)
    foreach(idl_name ${idl_names})
      set(idl_files ${idl_files} ${corba_src_dir}/${idl_name}.idl)
      set(idl_cpp_files ${idl_cpp_files} ${corba_binary_dir}/${idl_name}Sk.cpp ${corba_binary_dir}/${idl_name}DynSk.cpp)
      set(idl_h_files ${idl_h_files} ${corba_binary_dir}/${idl_name}.hh)
    endforeach()
    foreach(idl_name ${idl_names})
      if(UNIX)
	add_custom_command(
	  OUTPUT ${corba_binary_dir}/${idl_name}.hh ${corba_binary_dir}/${idl_name}DynSk.cpp ${corba_binary_dir}/${idl_name}Sk.cpp
	  COMMAND omniidl ${idl_flags} ${corba_src_dir}/${idl_name}.idl
	  DEPENDS ${idl_files}
	  COMMENT "Generating the C++ stubs and skeletons of ${idl_name}.idl"
	  )
      elseif(MSVC)
	add_custom_command(
	  OUTPUT ${corba_binary_dir}/${idl_name}.hh ${corba_binary_dir}/${idl_name}Sk.cpp ${corba_binary_dir}/${idl_name}DynSk.cpp
	  # The command path including spaces can only be invoked by the following variable expansion
	  COMMAND for %%A in \("${CHOREONOID_OMNIORB_BINARY_DIR}"\) do %%~sA\\omniidl ${idl_flags} ${corba_src_dir}/${idl_name}.idl
	  DEPENDS ${idl_files}
	  COMMENT "Generating the C++ stubs and skeletons of ${idl_name}.idl"
	  )
      endif()
    endforeach()
  else() # Make public
    # copy idl files to the system include directory
    set(corba_dir ${PROJECT_BINARY_DIR}/include/cnoid/${subdir})
    file(MAKE_DIRECTORY ${corba_dir})
    foreach(idl_name ${idl_names})
      set(idl_file ${corba_src_dir}/${idl_name}.idl)
      if(UNIX)
	add_custom_command(
	  OUTPUT ${corba_dir}/${idl_name}.idl
	  COMMAND cp ${idl_file} ${corba_dir}
	  DEPENDS ${idl_file}
	  COMMENT "Copying ${idl_name}.idl to ${corba_dir}"
	  )
      elseif(MSVC)
	file(TO_NATIVE_PATH ${corba_src_dir}/${idl_name}.idl src)
	file(TO_NATIVE_PATH ${corba_dir} dest)
	add_custom_command(
	  OUTPUT ${corba_dir}/${idl_name}.idl
	  COMMAND copy ${src} ${dest}
	  DEPENDS ${idl_file}
	  COMMENT "Copying ${idl_name}.idl to ${corba_dir}")
      endif()
      set(idl_files ${idl_files} ${corba_dir}/${idl_name}.idl)
      set(idl_cpp_files ${idl_cpp_files} ${corba_binary_dir}/${idl_name}Sk.cpp ${corba_binary_dir}/${idl_name}DynSk.cpp)
      set(idl_h_files ${idl_h_files} ${corba_dir}/${idl_name}.hh)
    endforeach()
    # idl compile
    foreach(idl_name ${idl_names})
      if(UNIX)
	add_custom_command(
	  OUTPUT ${corba_binary_dir}/${idl_name}.hh ${corba_dir}/${idl_name}.hh ${corba_binary_dir}/${idl_name}DynSk.cpp ${corba_binary_dir}/${idl_name}Sk.cpp
          COMMAND omniidl ${idl_flags} ${corba_dir}/${idl_name}.idl
          COMMAND cp ${corba_binary_dir}/${idl_name}.hh ${corba_dir}
          DEPENDS ${idl_files}
          COMMENT "Generating the C++ stubs and skeletons of ${idl_name}.idl"
          )
      elseif(MSVC)
	file(TO_NATIVE_PATH ${corba_binary_dir}/${idl_name}.hh src)
	file(TO_NATIVE_PATH ${corba_dir} dest)
	add_custom_command(
          OUTPUT ${corba_binary_dir}/${idl_name}.hh ${corba_dir}/${idl_name}.hh ${corba_binary_dir}/${idl_name}Sk.cpp ${corba_binary_dir}/${idl_name}DynSk.cpp
          COMMAND for %%A in \("${CHOREONOID_OMNIORB_BINARY_DIR}"\) do %%~sA\\omniidl ${idl_flags} ${corba_dir}/${idl_name}.idl
          COMMAND copy ${src} ${dest}
          DEPENDS ${idl_files}
          COMMENT "Generating the C++ stubs and skeletons of ${idl_name}.idl"
          )
      endif()
    endforeach()
    install(FILES ${idl_files} ${idl_h_files} DESTINATION ${CHOREONOID_HEADER_SUBDIR}/cnoid/${subdir})
  endif()

  set(${out_cpp_files} ${idl_cpp_files} PARENT_SCOPE)
  set(${out_header_files} ${idl_h_files} PARENT_SCOPE)

  set_source_files_properties(${idl_cpp_files} PROPERTIES GENERATED true COMPILE_FLAGS -DOMNI_UNLOADABLE_STUBS)
    
endfunction()

# Deprecated
function(idl_compile_cpp out_cpp_files out_h_files subdir)
  choreonoid_compile_corba_idl_to_cpp(cpp_files h_files ${subdir} ${ARGN})
  set(${out_cpp_files} ${cpp_files} PARENT_SCOPE)
  set(${out_h_files} ${h_files} PARENT_SCOPE)
endfunction()
