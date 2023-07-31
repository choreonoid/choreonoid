set(make_symlink false)

if(UNIX AND NOT CNOID_SOURCE_SHARE_DIR STREQUAL CNOID_BINARY_SHARE_DIR)
  set(make_symlink true)
endif()

# "default" directory
set(default_files materials.yaml)
foreach(file ${default_files})
  install(FILES default/${file} DESTINATION ${CNOID_SHARE_SUBDIR}/default)
endforeach()

if(make_symlink)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/default)
  foreach(file ${default_files})
    add_custom_target(
      symlink_to_default_${file} ALL
      COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CNOID_SOURCE_SHARE_DIR}/default/${file} ${CNOID_BINARY_SHARE_DIR}/default/${file})
  endforeach()
endif()

# Incremental build issue
if(IS_SYMLINK ${CNOID_BINARY_SHARE_DIR}/model)
  file(REMOVE ${CNOID_BINARY_SHARE_DIR}/model)
endif()

# "model" directory
if(make_symlink)
  file(MAKE_DIRECTORY ${CNOID_BINARY_SHARE_DIR}/model)
endif()
if(ENABLE_SAMPLES)
  install(DIRECTORY model DESTINATION ${CNOID_SHARE_SUBDIR})
  if(make_symlink)
    set(sample_model_dirs
      AizuSpider AizuWheel DoubleArmV7 GR001 HRP4C house JACO2 Labo1 multicopter
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
