
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

#set_source_files_properties(SceneWidget.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

set(sources
  App.cpp
  MenuManager.cpp
  Menu.cpp
  ProjectManager.cpp
  ParametricPathProcessor.cpp
  PathVariableEditor.cpp
  PluginManager.cpp
  MainWindow.cpp
  ViewArea.cpp
  ToolBarArea.cpp
  InfoBar.cpp
  ViewManager.cpp
  View.cpp
  MessageView.cpp
  ItemManager.cpp
  Item.cpp
  RootItem.cpp
  ItemPath.cpp
  ItemTreeView.cpp
  ItemPropertyView.cpp
  SelectionListEditor.cpp
  MultiSeqItemCreationPanel.cpp
  ToolBar.cpp
  TimeBar.cpp
  ScriptBar.cpp
  FileBar.cpp
  CaptureBar.cpp
  GraphWidget.cpp
  GraphBar.cpp
  GraphViewBase.cpp
  MultiValueSeqGraphView.cpp
  MultiSE3SeqGraphView.cpp
  Process.cpp
  Action.cpp
  ActionGroup.cpp
  SpinBox.cpp
  FloatingNumberBox.cpp
  ScrollBar.cpp
  Slider.cpp
  Splitter.cpp
  Buttons.cpp
  ButtonGroup.cpp
  CheckBox.cpp
  LineEdit.cpp
  ComboBox.cpp
  Dialog.cpp
  DescriptionDialog.cpp
  Timer.cpp
  SocketNotifier.cpp
  ImageWidget.cpp
  TreeView.cpp
  TreeWidget.cpp
  ItemSelectionModel.cpp
  ExtensionManager.cpp
  OptionManager.cpp
  Plugin.cpp
  FolderItem.cpp
  AbstractSeqItem.cpp
  MultiValueSeqItem.cpp
  MultiSE3SeqItem.cpp
  MultiAffine3SeqItem.cpp
  Vector3SeqItem.cpp
  ExtCommandItem.cpp
  AbstractTextItem.cpp
  ScriptItem.cpp
  TimeSyncItemEngine.cpp
  AppConfig.cpp
  Archive.cpp
  ItemTreeArchiver.cpp
  LazyCaller.cpp
  LazySignal.cpp
  Licenses.cpp
  GLSceneRenderer.cpp
  GL1SceneRenderer.cpp
  GLSLProgram.cpp
  ShaderPrograms.cpp
  GLSLSceneRenderer.cpp
  SceneWidget.cpp
  SceneWidgetEditable.cpp
  InteractiveCameraTransform.cpp
  SceneProjector.cpp
  SceneDragProjector.cpp
  SceneDragger.cpp
  TranslationDragger.cpp
  RotationDragger.cpp
  PositionDragger.cpp
  RectRegionMarker.cpp
  SceneView.cpp
  SceneBar.cpp
  SceneItem.cpp
  PointSetItem.cpp
  MultiPointSetItem.cpp
  MovieRecorder.cpp
  TextEditView.cpp
  ImageView.cpp
  TextEdit.cpp
  TaskView.cpp
  VirtualJoystickView.cpp
  gl_core_3_3.c
  #gl_core_4_0.c
  #gl_core_4_4.c
  gl_1_5.c
  )

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
  set(sources ${sources} JoystickCaptureLinux.cpp)
elseif(WIN32)
  #set(sources ${sources} JoystickCaptureWindows.cpp)
elseif(APPLE)
  #set(sources ${sources} JoystickCaptureOSX.cpp)
endif()

set(headers
  App.h
  MainWindow.h
  Process.h
  Action.h
  ActionGroup.h
  SpinBox.h
  FloatingNumberBox.h
  ScrollBar.h
  Slider.h
  Splitter.h
  Buttons.h
  ButtonGroup.h
  CheckBox.h
  LineEdit.h
  ComboBox.h
  Dialog.h
  Timer.h
  SocketNotifier.h
  ImageWidget.h
  TreeView.h
  TreeWidget.h
  ItemSelectionModel.h
  MenuManager.h
  Menu.h
  ToolBar.h
  View.h
  ViewArea.h
  ViewManager.h
  GraphWidget.h
  GraphViewBase.h
  ExtensionManager.h
  OptionManager.h
  ProjectManager.h
  PluginManager.h
  Plugin.h
  ParametricPathProcessor.h
  MessageView.h
  ItemTreeView.h
  ItemList.h
  Item.h
  PutPropertyFunction.h
  RootItem.h
  FolderItem.h
  AbstractSeqItem.h
  MultiSeqItem.h
  MultiValueSeqItem.h
  MultiSE3SeqItem.h
  MultiAffine3SeqItem.h
  Vector3SeqItem.h
  ExtCommandItem.h
  AbstractTextItem.h
  ScriptItem.h
  ItemPath.h
  TimeBar.h
  TimeSyncItemEngine.h
  GraphBar.h
  ItemManager.h
  AppConfig.h
  AppUtil.h
  Archive.h
  ItemTreeArchiver.h
  LazySignal.h
  LazyCaller.h
  SceneWidget.h
  SceneProjector.h
  SceneDragProjector.h
  SceneDragger.h
  TranslationDragger.h
  RotationDragger.h
  PositionDragger.h
  SceneBar.h
  SceneWidgetEditable.h
  InteractiveCameraTransform.h
  RectRegionMarker.h
  GLSceneRenderer.h
  GL1SceneRenderer.h
  GLSLSceneRenderer.h
  SceneView.h
  SceneItem.h
  PointSetItem.h
  MultiPointSetItem.h
  TextEditView.h
  ImageView.h
  JoystickCapture.h
  TextEdit.h
  TaskView.h
  exportdecl.h
  )

set(target CnoidBase)

set(qtheaders
  App.h
  Menu.h
  ToolBar.h
  Process.h
  Action.h
  ActionGroup.h
  SpinBox.h
  FloatingNumberBox.h
  ScrollBar.h
  Slider.h
  Splitter.h
  Buttons.h
  ButtonGroup.h
  CheckBox.h
  ComboBox.h
  LineEdit.h
  Dialog.h
  ImageWidget.h
  TreeView.h
  TreeWidget.h
  ItemSelectionModel.h
  Timer.h
  SocketNotifier.h
  InfoBar.h
  ItemTreeView.h
  SelectionListEditor.h
  TextEdit.h
  )

if(NOT QT5)
  QT4_ADD_RESOURCES(RC_SRCS Base.qrc)
  QT4_WRAP_CPP(sources
    ${qtheaders}
    OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
    )
else()
  QT5_ADD_RESOURCES(RC_SRCS Base.qrc)
  QT5_WRAP_CPP(sources
    ${qtheaders}
    OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
    )
endif()

make_gettext_mofiles(${target} mofiles)

add_cnoid_library(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})

if(QT5)
  qt5_use_modules(${target} Gui OpenGL Network Widgets)
  if(UNIX)
    qt5_use_modules(${target} X11Extras)
  endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OPENGL_LIBRARIES}
    intl
    )

elseif(UNIX)

  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OPENGL_LIBRARY}
    icuuc
    X11
    Xfixes
    )

elseif(MSVC)

  target_link_libraries(${target}
    ${QT_LIBRARIES}
    general glu32 opengl32
    CnoidUtil
    )
endif()

apply_common_setting_for_library(${target} "${headers}")

if(ENABLE_PYTHON)
  if(USE_PYBIND11)
    add_subdirectory(pybind11)
  elseif(USE_BOOST_PYTHON)
    add_subdirectory(boostpython)
  endif()
endif()

add_subdirectory(lua)
