function(CHOREONOID_SET_TARGET_COMMON_PROPERTIES target)
  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    target_compile_options(${target} PRIVATE "-fvisibility=hidden")
  endif()
  if(MSVC)
    target_link_options(${target} PRIVATE "/NODEFAULTLIB:LIBCMT")
    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX d)
  endif()
endfunction()

function(CHOREONOID_SET_HEADER_FILES)
  if(INSTALL_SDK OR MSVC_IDE)
    cmake_parse_arguments(ARG "INSTALL_HEADERS" "" "HEADERS" ${ARGV})
    if(NOT ARG_HEADERS)
      set(ARG_HEADERS "")
      foreach(file ${ARGN})
	get_filename_component(extension ${file} EXT)
	if(extension STREQUAL ".h")
	  list(APPEND ARG_HEADERS ${file})
	endif()
      endforeach()
    endif()
    if(ARG_HEADERS)
      if(MSVC_IDE)
	source_group("Header Files" FILES ${ARG_HEADERS})
      endif()
      if(INSTALL_SDK AND ARG_INSTALL_HEADERS)
	file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
	install(FILES ${ARG_HEADERS} DESTINATION ${CHOREONOID_HEADER_SUBDIR}/cnoid/${rel_src_dir})
      endif()
    endif()
  endif()
endfunction()

function(CHOREONOID_ADD_LIBRARY target)

  set(args ${ARGN})
  list(REMOVE_ITEM args HEADERS)
  add_library(${target} ${args})

  set_target_properties(${target} PROPERTIES VERSION ${CHOREONOID_VERSION})
  CHOREONOID_SET_TARGET_COMMON_PROPERTIES(${target})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN AND (ARGV1 STREQUAL "STATIC"))
    target_compile_options(${target} PRIVATE "-fPIC")
  endif()

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()

  CHOREONOID_SET_HEADER_FILES(${ARGN} INSTALL_HEADERS)

  if(ARGV1 STREQUAL "STATIC")
    if(INSTALL_SDK)
      install(TARGETS ${target}
        LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
        ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    endif()
  else()
    if(INSTALL_SDK)
      install(TARGETS ${target}
        RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
        LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
        ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    else()
      install(TARGETS ${target}
        RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
        LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    endif()
  endif()

endfunction()

function(CHOREONOID_ADD_PLUGIN target)

  set(add_library_args ${ARGN})
  list(REMOVE_ITEM add_library_args SHARED HEADERS)
  add_library(${target} SHARED ${add_library_args})

  CHOREONOID_SET_TARGET_COMMON_PROPERTIES(${target})

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR})

  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
  else()
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()

  CHOREONOID_SET_HEADER_FILES(${ARGN} INSTALL_HEADERS)

  if(INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()

endfunction()

function(CHOREONOID_ADD_EXECUTABLE target)

  add_executable(${target} ${ARGN})

  CHOREONOID_SET_TARGET_COMMON_PROPERTIES(${target})

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  endif()

  CHOREONOID_SET_HEADER_FILES(${ARGN})

  install(TARGETS ${target} RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()

function(CHOREONOID_MAKE_HEADER_PUBLIC)
  set(header_file ${ARGV0})
  if(ARGC EQUAL 1)
    get_filename_component(header ${header_file} NAME_WE)
  else()
    set(header ${ARGV1})
  endif()
  file(RELATIVE_PATH header_path ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${header_file})
  set(public_file ${PROJECT_BINARY_DIR}/include/cnoid/${header})
  file(WRITE ${public_file} "#include \"${header_path}\"\n")
  if(INSTALL_SDK)
    install(FILES ${public_file} DESTINATION ${CHOREONOID_HEADER_SUBDIR}/cnoid)
  endif()
endfunction()

function(CHOREONOID_MAKE_HEADERS_PUBLIC)
  foreach(header_file ${ARGV})
    CHOREONOID_MAKE_HEADER_PUBLIC(${header_file})
  endforeach()
endfunction()

function(CHOREONOID_MAKE_GETTEXT_MO_FILES target out_mo_files)
  set(${out_mo_files} "" PARENT_SCOPE)
  if(NOT ENABLE_GETTEXT)
    return()
  endif()
  file(GLOB pofiles ${CMAKE_CURRENT_SOURCE_DIR}/po/*.po)
  foreach(pofile ${pofiles})
    get_filename_component(lang ${pofile} NAME_WE)
    set(message_location share/locale/${lang}/LC_MESSAGES)
    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${message_location})
    set(mo_file ${PROJECT_BINARY_DIR}/${message_location}/${target}-${CHOREONOID_VERSION}.mo)
    add_custom_command(
      OUTPUT ${mo_file}
      COMMAND ${GETTEXT_MSGFMT_EXECUTABLE} -o ${mo_file} ${pofile}
      DEPENDS ${pofile}
      )
    list(APPEND mo_files ${mo_file})
    install(FILES ${mo_file} DESTINATION "share/locale/${lang}/LC_MESSAGES")
  endforeach()
  set(${out_mo_files} ${mo_files} PARENT_SCOPE)
endfunction()
