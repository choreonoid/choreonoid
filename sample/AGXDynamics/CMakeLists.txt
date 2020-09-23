if(NOT (BUILD_AGX_DYNAMICS_PLUGIN AND ENABLE_GUI))
  return()
endif()

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

if(UNIX)
  add_custom_target(
    symlink_to_agx_samples ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CNOID_BINARY_SHARE_DIR}/agx)
endif()

install(FILES ${models} DESTINATION ${CNOID_SHARE_SUBDIR}/agx)

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
  DESTINATION ${CNOID_SHARE_SUBDIR}/agx)
