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
	  COMMAND for %%A in \("${CHOREONOID_OMNIDYNAMIC_DIR}/bin/x86_win32"\) do %%~sA\\omniidl ${idl_flags} ${corba_src_dir}/${idl_name}.idl
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
          COMMAND for %%A in \("${CHOREONOID_OMNIDYNAMIC_DIR}/bin/x86_win32"\) do %%~sA\\omniidl ${idl_flags} ${corba_dir}/${idl_name}.idl
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

if(ENABLE_PYTHON)

  function(choreonoid_compile_corba_idl_to_python target src_subdir dest_subdir global_scope)

    set(args ${ARGV})
    list(REMOVE_AT args 0 1 2 3)
    set(is_dependencies FALSE)
    foreach(arg ${args})
      if(arg STREQUAL "DEPENDS")
	set(is_dependencies TRUE)
      else()
	if(is_dependencies)
	  set(dependencies ${dependencies} ${arg})
	else()
	  set(idl_names ${idl_names} ${arg})
	endif()
      endif()
    endforeach()

    set(package "")
    set(path ${dest_subdir})
    set(dir "dummy")
    while(path AND dir)
      get_filename_component(dir ${path} NAME)
      if(dir)
	if(package)
	  set(package "${dir}.${package}")
	else()
	  set(package ${dir})
	endif()
      endif()
      get_filename_component(path ${path} PATH)
    endwhile()

    set(python_dir ${PROJECT_BINARY_DIR}/${CHOREONOID_PYTHON_SUBDIR})
    set(output_dir ${python_dir}/${dest_subdir})
    file(MAKE_DIRECTORY ${output_dir})

    set(idl_flags -bpython -Wbglobal=${global_scope} -Wbpackage=${package} -I${PROJECT_SOURCE_DIR}/include)
    foreach(idl_include_dir ${IDL_INCLUDE_DIRS})
      set(idl_flags ${idl_flags} -I${idl_include_dir})
    endforeach()
    
    foreach(idl_name ${idl_names})
      set(idl_files ${idl_files} ${PROJECT_SOURCE_DIR}/include/cnoid/corba/${src_subdir}/${idl_name}.idl)
      set(outputs ${outputs} ${output_dir}/${idl_name}_idl.py)
    endforeach()

    set(prev_output)
    foreach(idl_name ${idl_names})
      set(idl_file ${PROJECT_SOURCE_DIR}/include/cnoid/corba/${src_subdir}/${idl_name}.idl)
      if(UNIX)
	add_custom_command(
	  OUTPUT ${output_dir}/${idl_name}_idl.py
	  COMMAND omniidl ${idl_flags} ${idl_file}
	  DEPENDS ${idl_files} ${dependencies} ${prev_output} # prev_output is necessary to make the compile sequential
	  WORKING_DIRECTORY ${python_dir}
	  )
      elseif(MSVC)
	add_custom_command(
	  OUTPUT ${output_dir}/${idl_name}_idl.py
	  COMMAND for %%A in \("${CHOREONOID_PYTHON_INCLUDE_PATH}/../bin/x86_win32"\) do %%~sA\\omniidl ${idl_flags} ${idl_file}
	  DEPENDS ${idl_files} ${dependencies}
	  WORKING_DIRECTORY ${python_dir}
	  )
      endif()
      set(prev_output ${output_dir}/${idl_name}_idl.py)
    endforeach()

    add_custom_target(${target} ALL DEPENDS ${outputs})

  endfunction()

  # Deprecated
  function(idl_compile_python target src_subdir dest_subdir global_scope)
    choreonoid_compile_corba_idl_to_python(${target} ${src_subdir} ${dest_subdir} ${global_scope})
  endfunction()

endif()
