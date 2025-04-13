if(NOT (BUILD_GL_CAMERA_EFFECT_PLUGIN AND ENABLE_SAMPLES))
  return()
endif()

set(models
  box.body
)

if(UNIX)
  add_custom_target(
    symlink_to_camera_effect_samples ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CNOID_BINARY_SHARE_DIR}/CameraEffect)
endif()

install(FILES ${models} DESTINATION ${CNOID_SHARE_SUBDIR}/CameraEffect)

install(FILES
  CameraEffect.cnoid
  DESTINATION ${CNOID_SHARE_SUBDIR}/CameraEffect)