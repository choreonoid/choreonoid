# @author Shin'ichiro Nakaoka

set(headers
  PyBase.h
  exportdecl.h
  )

add_cnoid_library(CnoidPyBase SHARED PyBase.cpp ${headers})
target_link_libraries(CnoidPyBase CnoidBase CnoidPyUtil)
apply_common_setting_for_library(CnoidPyBase "${headers}")

add_cnoid_python_module(PyQtCore PyQtCore.cpp)
target_link_libraries(PyQtCore ${QT_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})

add_cnoid_python_module(PyQtGui
  PyQtGui.cpp
  )

target_link_libraries(PyQtGui ${QT_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})

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
