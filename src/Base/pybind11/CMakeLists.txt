set(headers
  PyBase.h
  PyItemList.h
  PyQObjectHolder.h
  PyQString.h
  exportdecl.h
  )

choreonoid_add_library(CnoidPyBase SHARED PyItemList.cpp HEADERS ${headers})
target_link_libraries(CnoidPyBase CnoidBase CnoidPyUtil)

choreonoid_add_python_module(PyQtCore
  PyQtCore.cpp
  PyQtCoreQtNamespace.cpp
  )
target_link_libraries(PyQtCore ${CHOREONOID_QT_LIBRARIES} ${PYTHON_LIBRARIES})

choreonoid_add_python_module(PyQtGui
  PyQtGui.cpp
  )
target_link_libraries(PyQtGui ${CHOREONOID_QT_LIBRARIES} ${PYTHON_LIBRARIES})

choreonoid_add_python_module(PyQtWidgets
  PyQtWidgets.cpp
  PyQWidget.cpp
  PyQtLayoutClasses.cpp
  PyQtModelViewClasses.cpp
  )
target_link_libraries(PyQtWidgets ${CHOREONOID_QT_LIBRARIES} ${PYTHON_LIBRARIES})

choreonoid_add_python_module(PyBase
  PyBaseModule.cpp
  PyAppUtil.cpp
  PyQtEx.cpp
  PyItems.cpp
  PySeqItems.cpp
  PyMainWindow.cpp
  PyToolBars.cpp
  PyViews.cpp
  PyItemTreeView.cpp
  PySceneTypes.cpp
  PyLazyCaller.cpp
  PyPluginManager.cpp
  PyProjectManager.cpp
  PyExtensionManagers.cpp
)

target_link_libraries(PyBase CnoidPyBase)
