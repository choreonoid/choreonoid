# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
#set_source_files_properties(SimulatorItem.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(sources
  BodyPlugin.cpp
  WorldItem.cpp
  BodySelectionManager.cpp
  BodyItem.cpp
  BodyItemFileIO.cpp
  EditableSceneBody.cpp
  LinkKinematicsKitManager.cpp
  LinkOffsetFrameListItem.cpp
  MaterialTableItem.cpp
  SimulatorItem.cpp
  SubSimulatorItem.cpp
  ControllerItem.cpp
  SimpleControllerItem.cpp
  BodyMotionControllerItem.cpp
  ControllerLogItem.cpp
  BodyContactPointLoggerItem.cpp
  SimulationScriptItem.cpp
  AISTSimulatorItem.cpp
  KinematicSimulatorItem.cpp
  GLVisionSimulatorItem.cpp
  FisheyeLensConverter.cpp
  BodyMotionItem.cpp
  ZMPSeqItem.cpp
  MultiDeviceStateSeqItem.cpp
  WorldLogFileItem.cpp
  IoConnectionMapItem.cpp
  SensorVisualizerItem.cpp
  BodyTrackingCameraItem.cpp
  BodyMarkerItem.cpp
  BodySuperimposerAddon.cpp
  BodyMotionEngine.cpp
  BodyOverwriteAddon.cpp
  BodyElementOverwriteItem.cpp
  LinkShapeOverwriteItem.cpp
  DeviceOverwriteItem.cpp
  KinematicFaultChecker.cpp
  SplineFilterDialog.cpp
  BodyBar.cpp
  LeggedBodyBar.cpp
  KinematicsBar.cpp
  SimulationBar.cpp
  LinkDeviceTreeWidget.cpp
  LinkDeviceListView.cpp
  LinkPositionView.cpp
  LinkPositionWidget.cpp
  LinkPropertyView.cpp
  JointDisplacementView.cpp
  JointDisplacementWidget.cpp
  JointStateView.cpp
  BodyStateView.cpp
  DigitalIoDeviceView.cpp
  IoConnectionView.cpp
  JointGraphView.cpp
  LinkGraphView.cpp
  BodyLinkView.cpp
  HrpsysFileIO.cpp
  CollisionSeq.cpp
  CollisionSeqItem.cpp
  CollisionSeqEngine.cpp
  )

set(headers
  WorldItem.h
  BodySelectionManager.h
  BodyItem.h
  BodyItemFileIO.h
  EditableSceneBody.h
  LinkKinematicsKitManager.h
  LinkOffsetFrameListItem.h
  MaterialTableItem.h
  SimulatorItem.h
  SubSimulatorItem.h
  ControllerItem.h
  SimpleControllerItem.h
  ControllerLogItem.h
  BodyContactPointLoggerItem.h
  SimulationScriptItem.h
  AISTSimulatorItem.h
  KinematicSimulatorItem.h
  GLVisionSimulatorItem.h
  BodyMotionItem.h
  ZMPSeqItem.h
  MultiDeviceStateSeqItem.h
  WorldLogFileItem.h
  IoConnectionMapItem.h
  SensorVisualizerItem.h
  BodyTrackingCameraItem.h
  BodyMarkerItem.h
  BodySuperimposerAddon.h
  BodyOverwriteAddon.h
  BodyElementOverwriteItem.h
  LinkShapeOverwriteItem.h
  DeviceOverwriteItem.h
  KinematicFaultChecker.h
  BodyBar.h
  KinematicsBar.h
  SimulationBar.h
  LinkDeviceTreeWidget.h
  LinkDeviceListView.h
  LinkPositionView.h
  LinkPositionWidget.h
  JointDisplacementWidget.h
  CollisionSeq.h
  CollisionSeqItem.h
  CollisionSeqEngine.h
  AISTSimulatorItem.h
  exportdecl.h
  )

set(target CnoidBodyPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

qt5_add_resources(RC_SRCS BodyPlugin.qrc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

choreonoid_add_plugin(${target} ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})

set(boost_libraries ${Boost_IOSTREAMS_LIBRARY})
if(MSVC)
  set(boost_libraries ${boost_libraries} ${Boost_BZIP2_LIBRARY} ${Boost_ZLIB_LIBRARY})
endif()

target_link_libraries(${target} CnoidBody ${boost_libraries})

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
