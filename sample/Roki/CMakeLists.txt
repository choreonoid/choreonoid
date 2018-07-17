if(NOT (BUILD_ROKI_PLUGIN AND ENABLE_GUI))
  return()
endif()

add_cnoid_simple_controller(Arm2dofController Arm2dofController.cpp)

set(models
  arm_2dof.body
  breakWall.body
  )

file(COPY ${models} DESTINATION ${CNOID_BINARY_SHARE_DIR}/model/misc)
install(FILES ${models} DESTINATION ${CNOID_SHARE_SUBDIR}/model/misc)

install(FILES
  RokiArm2Dof.cnoid
  RokiFallingBoxes.cnoid
  RokiBreakWall.cnoid
  DESTINATION ${CNOID_SHARE_SUBDIR}/project)
