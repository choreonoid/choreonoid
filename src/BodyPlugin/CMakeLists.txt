#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
#set_source_files_properties(SimulatorItem.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(sources
  BodyPlugin.cpp
  WorldItem.cpp
  BodyWorldAddon.cpp
  BodySelectionManager.cpp
  BodyItem.cpp
  BodyItemFileIO.cpp
  OperableSceneBody.cpp
  BodyItemKinematicsKit.cpp
  BodyItemKinematicsKitManager.cpp
  LinkOffsetFrameListItem.cpp
  KinematicBodyItemSet.cpp
  BodyGeometryMeasurementTracker.cpp
  MaterialTableItem.cpp
  SimulatorItem.cpp
  SubSimulatorItem.cpp
  ControllerItem.cpp
  SimpleControllerItem.cpp
  BodyMotionControllerItem.cpp
  CollisionDetectionControllerItem.cpp
  RegionIntrusionDetectorItem.cpp
  BodyStateLoggerItem.cpp
  BodyContactPointLoggerItem.cpp
  BodyContactPointLogItem.cpp
  SimulationScriptItem.cpp
  AISTSimulatorItem.cpp
  KinematicSimulatorItem.cpp
  GLVisionSimulatorItem.cpp
  FisheyeLensConverter.cpp
  BodyMotionItem.cpp
  BodyMotionEngine.cpp
  MultiDeviceStateSeqItem.cpp
  MultiDeviceStateSeqEngine.cpp
  ZMPSeqItem.cpp
  WorldLogFileItem.cpp
  IoConnectionMapItem.cpp
  SensorVisualizerItem.cpp
  BodySyncCameraItem.cpp
  BodySyncCameraConfigDialog.cpp
  BodyMarkerItem.cpp
  BodySuperimposerAddon.cpp
  BodyOverwriteAddon.cpp
  BodyElementOverwriteItem.cpp
  LinkOverwriteItem.cpp
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
  JointDisplacementWidgetSet.cpp
  MultiBodyJointDisplacementWidget.cpp
  JointDisplacementView.cpp
  JointStateView.cpp
  BodyStateView.cpp
  DigitalIoDeviceView.cpp
  IoConnectionView.cpp
  BodyPositionGraphViewBase.cpp
  JointGraphView.cpp
  LinkGraphView.cpp
  BodyLinkView.cpp
  BodyLibraryView.cpp
  BodyLibrarySelectionDialog.cpp
  HrpsysFileIO.cpp
  CollisionSeq.cpp
  CollisionSeqItem.cpp
  CollisionSeqEngine.cpp
  )

set(headers
  WorldItem.h
  BodyWorldAddon.h
  BodySelectionManager.h
  BodyItem.h
  BodyItemFileIO.h
  OperableSceneBody.h
  BodyItemKinematicsKit.h
  BodyItemKinematicsKitManager.h
  LinkOffsetFrameListItem.h
  KinematicBodyItemSet.h
  MaterialTableItem.h
  SimulatorItem.h
  SubSimulatorItem.h
  ControllerItem.h
  SimpleControllerItem.h
  CollisionDetectionControllerItem.h
  RegionIntrusionDetectorItem.h
  ControllerLogItem.h
  BodyStateLoggerItem.h
  BodyContactPointLoggerItem.h
  BodyContactPointLogItem.h
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
  BodySyncCameraItem.h
  BodyMarkerItem.h
  BodySuperimposerAddon.h
  BodyOverwriteAddon.h
  BodyElementOverwriteItem.h
  LinkOverwriteItem.h
  DeviceOverwriteItem.h
  KinematicFaultChecker.h
  BodyBar.h
  KinematicsBar.h
  SimulationBar.h
  LinkDeviceTreeWidget.h
  LinkDeviceListView.h
  LinkPositionView.h
  LinkPositionWidget.h
  JointDisplacementWidgetSet.h
  MultiBodyJointDisplacementWidget.h
  IoConnectionView.h
  BodyLibraryView.h
  BodyLibrarySelectionDialog.h
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

target_link_libraries(${target} PUBLIC CnoidBody CnoidGLSceneRenderer PRIVATE ${boost_libraries})

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
