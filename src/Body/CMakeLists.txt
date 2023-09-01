#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(SceneBody.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(target CnoidBody)

set(sources
  Body.cpp
  Link.cpp
  ExtraJoint.cpp
  LinkTraverse.cpp
  LinkPath.cpp
  JointTraverse.cpp
  JointPath.cpp
  LinkGroup.cpp
  Jacobian.cpp
  BodyHandler.cpp
  BodyHandlerManager.cpp
  CustomJointPathBase.cpp
  JointSpaceConfigurationHandler.cpp
  LinkedJointHandler.cpp
  BodyCustomizerInterface.cpp
  Material.cpp
  ContactMaterial.cpp
  MaterialTable.cpp
  SceneBody.cpp
  SceneDevice.cpp
  AbstractBodyLoader.cpp
  BodyLoader.cpp
  StdBodyLoader.cpp
  StdBodyWriter.cpp
  ROSPackageSchemeHandler.cpp
  Device.cpp
  HolderDevice.cpp
  AttachmentDevice.cpp
  DigitalIoDevice.cpp
  IoConnectionMap.cpp
  ForceSensor.cpp
  RateGyroSensor.cpp
  AccelerationSensor.cpp
  Imu.cpp
  BasicSensorSimulationHelper.cpp
  VisionSensor.cpp
  Camera.cpp
  RangeCamera.cpp
  RangeSensor.cpp
  Light.cpp
  PointLight.cpp
  SpotLight.cpp
  MarkerDevice.cpp
  MultiDeviceStateSeq.cpp
  SceneCollision.cpp
  InverseKinematics.cpp
  CompositeIK.cpp
  PinDragIK.cpp
  BodyKinematicsKit.cpp
  KinematicBodySet.cpp
  LeggedBodyHelper.cpp
  BodyCollisionLinkFilter.cpp
  BodyCollisionDetector.cpp
  BodyCollisionDetectorUtil.cpp
  BodyMotion.cpp
  BodyPositionSeq.cpp
  BodyMotionPoseProvider.cpp
  BodyState.cpp
  ExtraBodyStateAccessor.cpp
  ZMPSeq.cpp
  ForwardDynamics.cpp
  ForwardDynamicsABM.cpp
  ForwardDynamicsCBM.cpp
  DyBody.cpp
  DyWorld.cpp
  MassMatrix.cpp
  ConstraintForceSolver.cpp
  InverseDynamics.cpp
  PenetrationBlocker.cpp
  VRMLBodyLoader.cpp
  VRMLBody.cpp
  PoseProviderToBodyMotionConverter.cpp
  BodyMotionUtil.cpp
  ControllerIO.cpp
  SimpleController.cpp
  CnoidBody.cpp # This file must be placed at the last position
  )

set(headers
  Body.h
  BodyHandler.h
  BodyHandlerManager.h
  CustomJointPathHandler.h
  CustomJointPathBase.h
  JointSpaceConfigurationHandler.h
  LinkedJointHandler.h
  BodyCustomizerInterface.h
  VRMLBody.h
  AbstractBodyLoader.h
  VRMLBodyLoader.h
  BodyLoader.h
  StdBodyLoader.h
  StdBodyWriter.h
  StdBodyFileUtil.h
  ZMPSeq.h
  Link.h
  ExtraJoint.h
  LinkTraverse.h
  LinkPath.h
  JointTraverse.h
  JointPath.h
  LinkGroup.h
  Material.h
  ContactMaterial.h
  MaterialTable.h
  BodyCollisionLinkFilter.h
  BodyCollisionDetector.h
  BodyCollisionDetectorUtil.h
  MultiDeviceStateSeq.h
  Device.h
  DeviceList.h
  HolderDevice.h
  AttachmentDevice.h
  DigitalIoDevice.h
  IoConnectionMap.h
  ForceSensor.h
  RateGyroSensor.h
  AccelerationSensor.h
  Imu.h
  BasicSensorSimulationHelper.h
  VisionSensor.h
  Camera.h
  RangeCamera.h
  RangeSensor.h
  Light.h
  PointLight.h
  SpotLight.h
  MarkerDevice.h
  SceneBody.h
  SceneDevice.h
  SceneCollision.h
  InverseKinematics.h
  CompositeIK.h
  CompositeBodyIK.h
  PinDragIK.h
  BodyKinematicsKit.h
  KinematicBodySet.h
  LeggedBodyHelper.h
  PenetrationBlocker.h
  ForwardDynamics.h
  ForwardDynamicsABM.h
  ForwardDynamicsCBM.h
  DyBody.h
  DyWorld.h
  InverseDynamics.h
  Jacobian.h
  MassMatrix.h
  ConstraintForceSolver.h
  PoseProvider.h
  BodyMotion.h
  BodyPositionSeq.h
  BodyMotionPoseProvider.h
  PoseProviderToBodyMotionConverter.h
  BodyMotionUtil.h
  BodyState.h
  ExtraBodyStateAccessor.h
  CollisionLinkPair.h
  ControllerIO.h
  SimpleController.h
  exportdecl.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_library(${target} SHARED ${sources} ${mofiles} HEADERS ${headers})

set(libraries PUBLIC CnoidUtil CnoidAISTCollisionDetector)
if(UNIX)
  set(libraries ${libraries} PRIVATE dl)
endif()
target_link_libraries(${target} ${libraries})

include(ChoreonoidBodyBuildFunctions.cmake)
if(CHOREONOID_INSTALL_SDK)
  install(FILES ChoreonoidBodyBuildFunctions.cmake DESTINATION ${CHOREONOID_CMAKE_CONFIG_SUBDIR}/ext)
endif()

set(BODY_CUSTOMIZERS ${BODY_CUSTOMIZERS} CACHE FILEPATH "Source files of body customizers")

if(BODY_CUSTOMIZERS)
  foreach(src ${BODY_CUSTOMIZERS})
    get_filename_component(customizer ${src} NAME_WE)
    choreonoid_add_body_customizer(${customizer} ${src})
  endforeach()
endif()

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
