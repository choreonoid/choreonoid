function(choreonoid_set_target_common_properties target)
  if(CHOREONOID_DEFAULT_FVISIBILITY_HIDDEN)
    target_compile_options(${target} PRIVATE "-fvisibility=hidden")
  endif()
  if(MSVC)
    target_link_options(${target} PRIVATE "/NODEFAULTLIB:LIBCMT")
    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX d)
  endif()
endfunction()

function(choreonoid_set_header_files)
  if(CHOREONOID_INSTALL_SDK OR MSVC_IDE)
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
      if(CHOREONOID_INSTALL_SDK AND ARG_INSTALL_HEADERS)
	file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
	install(FILES ${ARG_HEADERS} DESTINATION ${CHOREONOID_HEADER_SUBDIR}/cnoid/${rel_src_dir})
      endif()
    endif()
  endif()
endfunction()

function(choreonoid_make_header_public)
  set(header_file ${ARGV0})
  if(ARGC EQUAL 1)
    get_filename_component(header ${header_file} NAME_WE)
  else()
    set(header ${ARGV1})
  endif()
  file(RELATIVE_PATH header_path ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${header_file})
  set(public_file ${PROJECT_BINARY_DIR}/include/cnoid/${header})
  file(WRITE ${public_file} "#include \"${header_path}\"\n")
  if(CHOREONOID_INSTALL_SDK)
    install(FILES ${public_file} DESTINATION ${CHOREONOID_HEADER_SUBDIR}/cnoid)
  endif()
endfunction()

function(choreonoid_make_headers_public)
  foreach(header_file ${ARGV})
    choreonoid_make_header_public(${header_file})
  endforeach()
endfunction()

function(choreonoid_add_library target)

  set(args ${ARGN})
  list(REMOVE_ITEM args HEADERS)
  add_library(${target} ${args})

  set_target_properties(${target} PROPERTIES VERSION ${CHOREONOID_VERSION_MAJOR}.${CHOREONOID_VERSION_MINOR})
  choreonoid_set_target_common_properties(${target})

  if(ARGV1 STREQUAL "STATIC")
    set(is_static true)
  else()
    set(is_static false)
  endif()

  if(CHOREONOID_DEFAULT_FVISIBILITY_HIDDEN AND is_static)
    target_compile_options(${target} PRIVATE "-fPIC")
  endif()

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(CHOREONOID_ENABLE_INSTALL_RPATH AND (NOT is_static))
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()

  choreonoid_set_header_files(${ARGN} INSTALL_HEADERS)

  if(is_static)
    if(CHOREONOID_INSTALL_SDK)
      install(TARGETS ${target}
	LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    endif()
  else()
    if(CHOREONOID_INSTALL_SDK)
      install(TARGETS ${target}
        RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    else()
      install(TARGETS ${target}
        RUNTIME DESTINATION bin LIBRARY DESTINATION lib)
    endif()
  endif()

endfunction()

# Deprecated. Use choreonoid_add_library().
# If this function is used with apply_common_setting_for_library(${headers}),
# use the signature choreonoid_add_library(... HEADERS ${headers}) without using
# apply_common_setting_for_library.
function(add_cnoid_library)
  choreonoid_add_library(${ARGV})
endfunction()

# Deprecated. Use choreonoid_add_library with the HEADERS sigunature.
function(apply_common_setting_for_library target)
endfunction()

function(choreonoid_add_plugin target)

  set(add_library_args ${ARGN})
  list(REMOVE_ITEM add_library_args SHARED HEADERS)
  add_library(${target} SHARED ${add_library_args})

  choreonoid_set_target_common_properties(${target})

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR})

  if(CHOREONOID_ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
  endif()

  choreonoid_set_header_files(${ARGN} INSTALL_HEADERS)

  if(CHOREONOID_INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
      LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
      ARCHIVE DESTINATION lib)
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
      LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR})
  endif()

endfunction()

# Deprecated.
function(add_cnoid_plugin)
  choreonoid_add_plugin(${ARGV})
endfunction()

# Deprecated.
function(apply_common_setting_for_plugin target)
endfunction()

function(choreonoid_add_executable target)

  add_executable(${target} ${ARGN})

  choreonoid_set_target_common_properties(${target})

  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(CHOREONOID_ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  endif()

  choreonoid_set_header_files(${ARGN})

  install(TARGETS ${target} RUNTIME DESTINATION bin)

endfunction()

# Deprecated.
function(add_cnoid_executable)
  choreonoid_add_executable(${ARGV})
endfunction()

function(choreonoid_make_gettext_mo_files target out_mo_files)
  set(${out_mo_files} "" PARENT_SCOPE)
  if(NOT CHOREONOID_ENABLE_GETTEXT)
    return()
  endif()
  file(GLOB pofiles ${CMAKE_CURRENT_SOURCE_DIR}/po/*.po)
  foreach(pofile ${pofiles})
    get_filename_component(lang ${pofile} NAME_WE)
    set(message_location share/locale/${lang}/LC_MESSAGES)
    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${message_location})
    set(version "${CHOREONOID_VERSION_MAJOR}.${CHOREONOID_VERSION_MINOR}")
    set(mo_file ${PROJECT_BINARY_DIR}/${message_location}/${target}-${version}.mo)
    add_custom_command(
      OUTPUT ${mo_file}
      COMMAND ${CHOREONOID_GETTEXT_MSGFMT_EXECUTABLE} -o ${mo_file} ${pofile}
      DEPENDS ${pofile}
      )
    list(APPEND mo_files ${mo_file})
    install(FILES ${mo_file} DESTINATION "share/locale/${lang}/LC_MESSAGES")
  endforeach()
  set(${out_mo_files} ${mo_files} PARENT_SCOPE)
endfunction()

# Deprecated
function(cnoid_make_gettext_mofiles target out_mo_files)
  choreonoid_make_gettext_mo_files(${target} ${out_mo_files})
  set(${out_mo_files} ${${out_mo_files}} PARENT_SCOPE)
endfunction()

function(make_gettext_mofiles target out_mo_files)
  choreonoid_make_gettext_mo_files(${target} ${out_mo_files})
  set(${out_mo_files} ${${out_mo_files}} PARENT_SCOPE)
endfunction()
