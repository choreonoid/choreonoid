# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)
#set_source_files_properties(GLSLSceneRenderer.cpp PROPERTIES COMPILE_FLAGS "-O3 -g")

set(sources
  App.cpp
  MenuManager.cpp
  Menu.cpp
  ProjectManager.cpp
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
  ItemList.cpp
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
  Widget.cpp
  SpinBox.cpp
  FloatingNumberBox.cpp
  ScrollBar.cpp
  Slider.cpp
  Dial.cpp
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
  SubProjectItem.cpp
  AbstractSeqItem.cpp
  MultiValueSeqItem.cpp
  MultiSE3SeqItem.cpp
  MultiSE3MatrixSeqItem.cpp
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
  SceneWidgetEvent.cpp
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
  GeneralSliderView.cpp
  ImageView.cpp
  TextEdit.cpp
  TaskView.cpp
  VirtualJoystickView.cpp
  MessageLogItem.cpp
  LightingItem.cpp
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
  Widget.h
  SpinBox.h
  FloatingNumberBox.h
  ScrollBar.h
  Slider.h
  Dial.h
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
  MessageView.h
  ItemTreeView.h
  ItemList.h
  Item.h
  PutPropertyFunction.h
  RootItem.h
  FolderItem.h
  SubProjectItem.h
  AbstractSeqItem.h
  MultiSeqItem.h
  MultiValueSeqItem.h
  MultiSE3SeqItem.h
  MultiSE3MatrixSeqItem.h
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
  SceneWidgetEvent.h
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
  GeneralSliderView.h
  ImageView.h
  JoystickCapture.h
  TextEdit.h
  TaskView.h
  MessageLogItem.h
  LightingItem.h
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
  Widget.h
  SpinBox.h
  FloatingNumberBox.h
  ScrollBar.h
  Slider.h
  Dial.h
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

QT5_ADD_RESOURCES(RC_SRCS Base.qrc)
QT5_WRAP_CPP(qtsources ${qtheaders})

set(sources ${sources} ${qtsources})

make_gettext_mofiles(${target} mofiles)

add_cnoid_library(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OPENGL_LIBRARIES}
    intl
    )
elseif(MSVC)
  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES}
    general glu32 opengl32
    )
else()
  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OPENGL_LIBRARY}
    icuuc X11 Xfixes
    )
endif()

apply_common_setting_for_library(${target} "${headers}")

if(ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()

add_subdirectory(lua)
