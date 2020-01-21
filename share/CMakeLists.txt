set(make_symlink false)
if(NOT CNOID_SOURCE_SHARE_DIR STREQUAL CNOID_BINARY_SHARE_DIR)
  if(UNIX)
    set(make_symlink true)
  endif()
  configure_file(default/materials.yaml ${CNOID_BINARY_SHARE_DIR}/default/materials.yaml COPYONLY)
endif()

install(FILES default/materials.yaml DESTINATION ${CNOID_SHARE_SUBDIR}/default)

set(symlink_directories model)
foreach(dir ${symlink_directories})
  install(DIRECTORY ${dir} DESTINATION ${CNOID_SHARE_SUBDIR})
  if(make_symlink)
    add_custom_target(
      symlink_to_${dir} ALL
      COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CNOID_SOURCE_SHARE_DIR}/${dir} ${CNOID_BINARY_SHARE_DIR}/${dir})
  endif()
endforeach()
