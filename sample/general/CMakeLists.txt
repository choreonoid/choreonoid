choreonoid_add_body_customizer(SpringModelCustomizer SpringModelCustomizer.cpp)

install(FILES
  FallingBoxes.cnoid
  FallingBoxesContactPoints.cnoid
  Blocks.cnoid
  ClosedLinkSample.cnoid
  ExtraJointSample.cnoid
  ConcaveShapeTest.cnoid
  Ragdoll.cnoid
  CustomizedSpringModel.cnoid
  DESTINATION ${CNOID_SHARE_SUBDIR}/project)
