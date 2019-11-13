if(NOT (BUILD_AGX_DYNAMICS_PLUGIN AND ENABLE_GUI))
  return()
endif()

return()

set(models
  BreachingPlate.body
  LockedDoor.body
  agxMaterialSample.body
  agxWireSample.body
  FireHose.body
  agxMagneticJoint.body
  agxValve.body
  vmark.body
  )

file(COPY ${models} DESTINATION ${CNOID_BINARY_SHARE_DIR}/model/misc)
install(FILES ${models} DESTINATION ${CNOID_SHARE_SUBDIR}/model/misc)

install(FILES
  AizuSpiderNA.cnoid
  AizuSpiderSA.cnoid
  AizuSpiderDA.cnoid
  DoubleArmV7A.cnoid
  AgxTank.cnoid
  ConvexDecomposition.cnoid
  LockedDoor.cnoid
  agxMagneticJoint.cnoid
  agxValve.cnoid
  FireHose.cnoid
  agxBreaching.cnoid
  agxMaterialSample.cnoid agxWireSample.cnoid
  DESTINATION ${CNOID_SHARE_SUBDIR}/project)
