# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(SceneBody.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(target CnoidBody)

set(sources
  Body.cpp
  Link.cpp
  LinkTraverse.cpp
  LinkPath.cpp
  JointPath.cpp
  Jacobian.cpp
  BodyHandler.cpp
  BodyHandlerManager.cpp
  CustomJointPathBase.cpp
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
  Device.cpp
  HolderDevice.cpp
  AttachmentDevice.cpp
  DigitalIoDevice.cpp
  IoConnectionMap.cpp
  ForceSensor.cpp
  RateGyroSensor.cpp
  AccelerationSensor.cpp
  BasicSensorSimulationHelper.cpp
  Camera.cpp
  RangeCamera.cpp
  RangeSensor.cpp
  Light.cpp
  PointLight.cpp
  SpotLight.cpp
  MarkerDevice.cpp
  MultiDeviceStateSeq.cpp
  ExtraBodyStateAccessor.cpp
  SceneCollision.cpp
  InverseKinematics.cpp
  CompositeIK.cpp
  PinDragIK.cpp
  LinkKinematicsKit.cpp
  LinkGroup.cpp
  LeggedBodyHelper.cpp
  BodyCollisionDetector.cpp
  BodyCollisionDetectorUtil.cpp
  BodyMotion.cpp
  BodyMotionPoseProvider.cpp
  BodyState.cpp
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
  BodyCustomizerInterface.h
  VRMLBody.h
  AbstractBodyLoader.h
  VRMLBodyLoader.h
  BodyLoader.h
  StdBodyLoader.h
  StdBodyWriter.h
  ZMPSeq.h
  Link.h
  LinkTraverse.h
  LinkPath.h
  JointPath.h
  LinkGroup.h
  Material.h
  ContactMaterial.h
  MaterialTable.h
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
  BasicSensorSimulationHelper.h
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
  LinkKinematicsKit.h
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
  BodyMotionPoseProvider.h
  PoseProviderToBodyMotionConverter.h
  BodyMotionUtil.h
  BodyState.h
  CollisionLinkPair.h
  ExtraJoint.h
  ControllerIO.h
  SimpleController.h
  exportdecl.h
  CnoidBody.h
  )

choreonoid_make_gettext_mo_files(${target} mofiles)
choreonoid_add_library(${target} SHARED ${sources} ${mofiles} HEADERS ${headers})

if(UNIX)
  target_link_libraries(${target} CnoidUtil CnoidAISTCollisionDetector dl)
elseif(MSVC)
  target_link_libraries(${target} CnoidUtil CnoidAISTCollisionDetector)
endif()

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

# TODO: Remove the following code. It is written in CHOREONID_BODY_ADD_SIMPLE_CONTROLLER
#file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller)
#install(
#  DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
#  DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller
#  FILES_MATCHING PATTERN "*" EXCLUDE)

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
