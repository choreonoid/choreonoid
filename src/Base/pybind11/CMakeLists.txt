# @author Shin'ichiro Nakaoka

set(headers
  PyItemList.h
  PyQString.h
  exportdecl.h
  )

add_cnoid_library(CnoidPyBase SHARED PyItemList.cpp)
target_link_libraries(CnoidPyBase CnoidBase CnoidPython)
apply_common_setting_for_library(CnoidPyBase "${headers}")

add_cnoid_python_module(PyQtCore PyQtCore.cpp)
target_link_libraries(PyQtCore ${QT_LIBRARIES} ${PYTHON_LIBRARIES})

if(QT5)
  qt5_use_modules(PyQtCore Core)
endif()

add_cnoid_python_module(PyQtGui
  PyQtGui.cpp
  )

if(QT5)
  qt5_use_modules(PyQtGui Widgets)
endif()

target_link_libraries(PyQtGui ${QT_LIBRARIES} ${PYTHON_LIBRARIES})

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
)

target_link_libraries(PyBase CnoidPyBase)
