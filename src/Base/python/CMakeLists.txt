set(headers
  PyBase.h
  PyItemList.h
  PyQString.h
  PyQtSignal.h
  exportdecl.h
  )

choreonoid_add_library(CnoidPyBase SHARED PyItemList.cpp HEADERS ${headers})
# PyItemList.cpp calls the Python C API directly, so link libpython explicitly.
target_link_libraries(CnoidPyBase PUBLIC CnoidBase ${PYTHON_LIBRARIES})
# PyItemList.h / PyQString.h include <nanobind/nanobind.h>, so consumers need it.
target_include_directories(CnoidPyBase PUBLIC
  ${PROJECT_SOURCE_DIR}/thirdparty/nanobind-2.12.0/include)
# PyItemList.cpp uses the nanobind runtime (nb::cast, python_error, ...), so the
# library must link the shared nanobind core (built via NB_SHARED). The target
# name depends on the free-threaded build option.
if(CHOREONOID_PYTHON_FREE_THREADED)
  target_link_libraries(CnoidPyBase PUBLIC nanobind-ft)
else()
  target_link_libraries(CnoidPyBase PUBLIC nanobind)
endif()

choreonoid_add_python_module(PyQtCore
  PyQtCore.cpp
  PyQtCoreQtNamespace.cpp
  )
target_link_libraries(PyQtCore PRIVATE ${CHOREONOID_QT_LIBRARIES})

choreonoid_add_python_module(PyQtGui
  PyQtGui.cpp
  )
target_link_libraries(PyQtGui PRIVATE ${CHOREONOID_QT_LIBRARIES})

choreonoid_add_python_module(PyQtWidgets
  PyQtWidgets.cpp
  PyQWidget.cpp
  PyQtLayoutClasses.cpp
  PyQtModelViewClasses.cpp
  )
target_link_libraries(PyQtWidgets PRIVATE ${CHOREONOID_QT_LIBRARIES})

choreonoid_add_python_module(PyBase
  PyBaseModule.cpp
  PyArchive.cpp
  PyApp.cpp
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
target_link_libraries(PyBase PRIVATE CnoidPyBase)
