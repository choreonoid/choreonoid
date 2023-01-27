choreonoid_add_body_handler(ClosedLinkSampleHandler ClosedLinkSampleHandler.cpp)
choreonoid_add_body_customizer(SpringModelCustomizer SpringModelCustomizer.cpp)

install_project_files(
  FallingBoxes.cnoid
  FallingBoxesContactPoints.cnoid
  Blocks.cnoid
  ClosedLinkSample.cnoid
  ExtraJointSample.cnoid
  ExtraJoint.py
  ConcaveShapeTest.cnoid
  Ragdoll.cnoid
  CustomizedSpringModel.cnoid
  )
