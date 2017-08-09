# @author Shin'ichiro Nakaoka

return()

if(NOT ENABLE_PYTHON3)
  return()
endif()

add_cnoid_library(CnoidPy3Base SHARED PyBase.cpp)
target_link_libraries(CnoidPy3Base CnoidBase CnoidPython)
apply_common_setting_for_library(CnoidPy3Base)

add_cnoid_python3_module(Py3QtCore
  PyQtCore.cpp
  )

target_link_libraries(Py3QtCore
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${Boost_PYTHON_LIBRARY}
  )

if(QT5)
  qt5_use_modules(Py3QtCore Core)
endif()

add_cnoid_python3_module(Py3QtGui
  PyQtGui.cpp
  )

if(QT5)
  qt5_use_modules(Py3QtGui Widgets)
endif()

target_link_libraries(Py3QtGui
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  )

add_cnoid_python3_module(Py3Base
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

target_link_libraries(Py3Base CnoidPy3Base)

set(headers
  PyBase.h
  exportdecl.h
  )

apply_common_setting_for_python_module(Py3Base "${headers}")
