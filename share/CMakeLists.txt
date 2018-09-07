if(NOT CNOID_SOURCE_SHARE_DIR STREQUAL CNOID_BINARY_SHARE_DIR)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/default)
  file(COPY default/materials.yaml DESTINATION ${CNOID_BINARY_SHARE_DIR}/default)
  # Make the symbolic link to the model directory
  if(UNIX)
    add_custom_target(
      symlink_to_model ALL
      COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CNOID_SOURCE_SHARE_DIR}/model ${CNOID_BINARY_SHARE_DIR}/model)
  endif()
endif()

install(FILES default/materials.yaml DESTINATION ${CNOID_SHARE_SUBDIR}/default)
install(DIRECTORY model DESTINATION ${CNOID_SHARE_SUBDIR})
