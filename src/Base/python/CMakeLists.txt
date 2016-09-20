
# @author Shin'ichiro Nakaoka

set(PYTHON_SUFFIX -python2.7)

add_cnoid_library(CnoidPyBase SHARED PyBase.cpp)
target_link_libraries(CnoidPyBase CnoidBase CnoidPython)
apply_common_setting_for_library(CnoidPyBase)

add_cnoid_python_module(PyQtCore
  PyQtCore.cpp
  )

target_link_libraries(PyQtCore
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${Boost_PYTHON_LIBRARY}
  )

if(QT5)
  qt5_use_modules(PyQtCore Core)
endif()

add_cnoid_python_module(PyQtGui
  PyQtGui.cpp
  )

if(QT5)
  qt5_use_modules(PyQtGui Widgets)
endif()

target_link_libraries(PyQtGui
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${Boost_PYTHON_LIBRARY}
  )


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

set(headers
  PyBase.h
  exportdecl.h
  )

apply_common_setting_for_python_module(PyBase "${headers}")
