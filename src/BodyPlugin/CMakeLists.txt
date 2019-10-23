# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
#set_source_files_properties(SimulatorItem.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(sources
  BodyPlugin.cpp
  WorldItem.cpp
  BodySelectionManager.cpp
  BodyItem.cpp
  LinkCoordinateFrameListSetItem.cpp
  LinkKinematicsKitManager.cpp
  SimulatorItem.cpp
  SubSimulatorItem.cpp
  ControllerItem.cpp
  SimpleControllerItem.cpp
  BodyMotionControllerItem.cpp
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
  BodyMotionEngine.cpp
  KinematicFaultChecker.cpp
  SplineFilterDialog.cpp
  EditableSceneBody.cpp
  BodyBar.cpp
  LeggedBodyBar.cpp
  KinematicsBar.cpp
  SimulationBar.cpp
  PositionView.cpp
  LinkTreeWidget.cpp
  LinkSelectionView.cpp
  LinkPropertyView.cpp
  JointDisplacementView.cpp
  JointStateView.cpp
  BodyStateView.cpp
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
  LinkCoordinateFrameListSetItem.h
  LinkKinematicsKitManager.h
  SimulatorItem.h
  SubSimulatorItem.h
  ControllerItem.h
  SimpleControllerItem.h
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
  KinematicFaultChecker.h
  BodyBar.h
  KinematicsBar.h
  SimulationBar.h
  PositionView.h
  LinkTreeWidget.h
  LinkSelectionView.h
  EditableSceneBody.h
  CollisionSeq.h
  CollisionSeqItem.h
  CollisionSeqEngine.h
  AISTSimulatorItem.h
  exportdecl.h
  )

set(target CnoidBodyPlugin)

make_gettext_mofiles(${target} mofiles)

QT5_ADD_RESOURCES(RC_SRCS BodyPlugin.qrc)
QT5_WRAP_CPP(qtsources LinkTreeWidget.h)
set(sources ${sources} ${qtsources})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})

set(boost_libraries ${Boost_IOSTREAMS_LIBRARY})
if(MSVC)
  set(boost_libraries ${boost_libraries} ${Boost_BZIP2_LIBRARY} ${Boost_ZLIB_LIBRARY})
endif()

target_link_libraries(${target} CnoidBase CnoidBody ${boost_libraries})
apply_common_setting_for_plugin(${target} "${headers}")

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
