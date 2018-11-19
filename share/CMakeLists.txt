if(NOT CNOID_SOURCE_SHARE_DIR STREQUAL CNOID_BINARY_SHARE_DIR)
  set(copy_share_dir TRUE)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/default)
  file(COPY default/materials.yaml DESTINATION ${CNOID_BINARY_SHARE_DIR}/default)
endif()

if(${CMAKE_VERSION} VERSION_GREATER "3.3.0") # check if IN_LIST is supported
  set(symlink_exclude_list default icon locale motion CMakeFiles)
  file(GLOB subdirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} LIST_DIRECTORIES true "*")
  foreach(subdir ${subdirs})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
      if(NOT subdir IN_LIST symlink_exclude_list)
	install(DIRECTORY ${subdir} DESTINATION ${CNOID_SHARE_SUBDIR})
	# Make the symbolic link to the sub directory
	if(UNIX AND copy_share_dir)
	  add_custom_target(
	    symlink_to_${subdir} ALL
	    COMMAND ${CMAKE_COMMAND} -E create_symlink
	    ${CNOID_SOURCE_SHARE_DIR}/${subdir} ${CNOID_BINARY_SHARE_DIR}/${subdir})
	endif()
      endif()
    endif()
  endforeach()
endif()

install(FILES default/materials.yaml DESTINATION ${CNOID_SHARE_SUBDIR}/default)
