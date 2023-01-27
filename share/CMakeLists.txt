set(make_symlink false)

if(UNIX AND NOT CNOID_SOURCE_SHARE_DIR STREQUAL CNOID_BINARY_SHARE_DIR)
  set(make_symlink true)
endif()

# "default" directory
install(FILES default/materials.yaml DESTINATION ${CNOID_SHARE_SUBDIR}/default)
if(make_symlink)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/default)
  add_custom_target(
    symlink_to_default_materials ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CNOID_SOURCE_SHARE_DIR}/default/materials.yaml ${CNOID_BINARY_SHARE_DIR}/default/materials.yaml)
endif()

# "model" directory
install(DIRECTORY model DESTINATION ${CNOID_SHARE_SUBDIR})
if(make_symlink)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/model)
  if(ENABLE_SAMPLES)
    set(sample_model_dirs
      AizuSpider AizuWheel DoubleArmV7 GR001 house JACO2 Labo1 multicopter
      PA10 RIC30 Robotiq SR1 Tank UniversalRobots WAREC1
      misc)
    foreach(dir ${sample_model_dirs})
      add_custom_target(
	symlink_to_model_${dir} ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CNOID_SOURCE_SHARE_DIR}/model/${dir} ${CNOID_BINARY_SHARE_DIR}/model/${dir})
    endforeach()
  endif()
endif()

# "project" directory
if(make_symlink)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/project)
endif()
