# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(SceneBody.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(target CnoidBody)

set(sources
  Body.cpp
  Link.cpp
  Device.cpp
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
  YAMLBodyLoader.cpp
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
  CompositeIK.cpp
  PinDragIK.cpp
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

make_gettext_mofiles(${target} mofiles)
add_cnoid_library(${target} SHARED ${sources} ${headers} ${mofiles})

if(UNIX)
  target_link_libraries(${target} CnoidUtil CnoidAISTCollisionDetector dl)
elseif(MSVC)
  target_link_libraries(${target} CnoidUtil CnoidAISTCollisionDetector)
endif()

apply_common_setting_for_library(${target} "${headers}")

# Body handler
function(add_cnoid_body_handler)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  set_target_properties(${target} PROPERTIES
    PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/bodyhandler)
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()
  target_link_libraries(${target} CnoidBody)
  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/bodyhandler CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/bodyhandler CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
endfunction()

# Body customizer (deprecated)
function(add_cnoid_body_customizer)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  set_target_properties(${target} PROPERTIES
    PREFIX ""
    COMPILE_DEFINITIONS "CNOID_BODY_CUSTOMIZER"
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/customizer)
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()
  target_link_libraries(${target} CnoidBase)
  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
endfunction()

set(BODY_CUSTOMIZERS ${BODY_CUSTOMIZERS} CACHE FILEPATH "Source files of body customizers")

if(BODY_CUSTOMIZERS)
  foreach(src ${BODY_CUSTOMIZERS})
    get_filename_component(customizer ${src} NAME_WE)
    add_cnoid_body_customizer(${customizer} ${src})
  endforeach()
endif()

# SimpleController
function(add_cnoid_simple_controller)

  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)

  add_library(${target} SHARED ${ARGV})

  target_link_libraries(${target} CnoidBody)

  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
    PREFIX "")
  if(ENABLE_INSTALL_RPATH)
    set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN")
  endif()

  install(TARGETS ${target}
    RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller)

install(
  DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/simplecontroller
  DESTINATION ${CNOID_PLUGIN_SUBDIR}/simplecontroller
  FILES_MATCHING PATTERN "*" EXCLUDE)

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
