
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

#set_source_files_properties(ToolBarArea.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")
#set_source_files_properties(ToolBar.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")
#set_source_files_properties(GLSceneRenderer.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")
#set_source_files_properties(SceneItem.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")
#set_source_files_properties(MessageView.cpp PROPERTIES COMPILE_FLAGS "-O0 -g")

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
  Slider.cpp
  ScrollBar.cpp
  Button.cpp
  ButtonGroup.cpp
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
  AppUtil.cpp
  Archive.cpp
  ItemTreeArchiver.cpp
  LazyCaller.cpp
  LazySignal.cpp
  Licenses.cpp
  SceneWidget.cpp
  SceneBar.cpp
  SceneWidgetEditable.cpp
  SceneProjector.cpp
  SceneDragProjector.cpp
  SceneDragger.cpp
  GLSceneRenderer.cpp
  SceneView.cpp
  SceneItem.cpp
  PointSetItem.cpp
  MultiPointSetItem.cpp
  MovieGenerator.cpp
  TextEditView.cpp
  ImageView.cpp
  TextEdit.cpp
  TaskView.cpp
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
  Slider.h
  ScrollBar.h
  Button.h
  ButtonGroup.h
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
  SceneBar.h
  SceneWidgetEditable.h
  GLSceneRenderer.h
  SceneView.h
  SceneItem.h
  PointSetItem.h
  MultiPointSetItem.h
  TextEditView.h
  ImageView.h
  JoystickCapture.h
  exportdecl.h
  gettext.h
  TextEdit.h
  TaskView.h
  )

set(target CnoidBase)

if(NOT QT5)
  QT4_ADD_RESOURCES(RC_SRCS Base.qrc)
else()
  QT5_ADD_RESOURCES(RC_SRCS Base.qrc)
endif()

if(NOT QT5)
QT4_WRAP_CPP(sources
  App.h
  Menu.h
  ToolBar.h
  Process.h
  Action.h
  ActionGroup.h
  SpinBox.h
  FloatingNumberBox.h
  Slider.h
  ScrollBar.h
  Button.h
  ButtonGroup.h
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
  TextEditView.h
  TextEdit.h
  OPTIONS "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED"
  )
endif()

make_gettext_mofiles(${target} mofiles)

add_cnoid_library(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})

if(QT5)
  qt5_use_modules(${target} Gui OpenGL Network Widgets)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}
    intl
    )

elseif(UNIX)

  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OPENGL_LIBRARY} ${GLEW_LIBRARIES}
    icuuc
    GLEW
    )

elseif(MSVC)

  target_link_libraries(${target}
    ${QT_LIBRARIES}
    general glu32
    CnoidUtil
    ${GLEW_LIBRARIES}
    )
endif()

apply_common_setting_for_library(${target} "${headers}")


if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()
