# @author Shin'ichiro Nakaoka

set(headers
  PyItemList.h
  PyQString.h
  exportdecl.h
  )

add_cnoid_library(CnoidPyBase SHARED PyItemList.cpp)
target_link_libraries(CnoidPyBase CnoidBase CnoidPyUtil)
apply_common_setting_for_library(CnoidPyBase "${headers}")

add_cnoid_python_module(PyQtCore PyQtCore.cpp)
target_link_libraries(PyQtCore ${QT_LIBRARIES} ${PYTHON_LIBRARIES})
apply_common_setting_for_python_module(PyQtCore)

add_cnoid_python_module(PyQtGui PyQtGui.cpp)
target_link_libraries(PyQtGui ${QT_LIBRARIES} ${PYTHON_LIBRARIES})
apply_common_setting_for_python_module(PyQtGui)

add_cnoid_python_module(PyBase
  PyBaseModule.cpp
  PyQtEx.cpp
  PyItems.cpp
  PyMainWindow.cpp
  PyToolBars.cpp
  PyViews.cpp
  PyItemTreeView.cpp
  PySceneTypes.cpp
  PyLazyCaller.cpp
  PyPluginManager.cpp
  PyProjectManager.cpp
)
target_link_libraries(PyBase CnoidPyBase)
apply_common_setting_for_python_module(PyBase)
